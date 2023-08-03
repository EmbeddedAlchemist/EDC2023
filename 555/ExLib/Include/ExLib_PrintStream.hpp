#pragma once

#include <limits>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <type_traits>

// #include "ExLib_Printable.hpp"
#include "ExLib_WriteStream.hpp"

namespace ExLib {

class PrintStream;

/**
 * @brief 当一个类继承Printable并重写了printTo方法，则称这个类是“可打印的”
 * @brief 当使用一个PrintStream打印一个Printable时，Printable的printTo方法会被调用
 * @brief 在重写的printTo方法中，你需要使用PrintStream提供的接口完成打印，并返回打印的字符数
 *
 */
class Printable {
  protected:
    friend class PrintStream;
    /**
     * @brief 输出到指定的打印流
     *
     * @param stream 指定的打印流
     * @return size_t 输出的字符数
     */
    virtual std::size_t printTo(PrintStream &stream) = 0;
};

class Printable;

/**
 * @brief 类可以通过继承PrintStream并至少重写bool write(char)方法来实现流式打印
 * @brief 如果对数据块的输出有更好的性能，可以重写std::size_t write(const char *buf, std::size_t len)来使用这一特性。若不重写，则默认调用write(char)方法输出
 * @brief 如果在某些情况下不可写入，则可重写bool avaliableForWrite(void)来为不可写入状态提供信号。
 *
 */
class PrintStream : public WriteStream {

  private:
    template <typename Interger>
    std::size_t printInteger(Interger integer, std::uint8_t base);

    template <typename Integer>
    std::size_t getIntegerDigits(Integer integer, std::uint8_t base);

    template <typename Floating>
    std::size_t printFloat(Floating floating, std::uint8_t tailingDigits);

    template <typename Floating>
    std::size_t getFloatIntegerDigits(Floating floating);

    char getNumberCharInBase(std::uint8_t number, std::uint8_t base);

  public:
    inline virtual ~PrintStream(void) {
    }

    template <typename Integer, typename std::enable_if<std::is_integral<Integer>::value, bool>::type = false>
    inline std::size_t print(Integer integer, std::uint8_t base = 10) { return printInteger(integer, base); }

    template <typename Floating, typename std::enable_if<std::is_floating_point<Floating>::value, int>::type = 0>
    inline std::size_t print(Floating floating, std::uint8_t tailingDigits = 2) { return printFloat(floating, tailingDigits); }

    std::size_t print(Printable &printable) { return printable.printTo(*this); }

    inline std::size_t print(const char *str) { return writeUntil(str, '\0'); }

    inline std::size_t println(void) { return write('\n') ? 0 : 1; }

    template <typename Integer, typename std::enable_if<std::is_integral<Integer>::value, bool>::type = false>
    inline std::size_t println(Integer integer, std::uint8_t base = 10) { return print(integer, base) + (write('\n') ? 0 : 1); }

    template <typename Floating, typename std::enable_if<std::is_floating_point<Floating>::value, int>::type = 0>
    inline std::size_t println(Floating floating, std::uint8_t tailingDigits = 2) { return print(floating, tailingDigits) + (write('\n') ? 0 : 1); }

    inline std::size_t println(Printable &printable) { return printable.printTo(*this) + (write('\n') ? 0 : 1); }

    inline std::size_t println(const char *str) { return print(str) + (write('\n') ? 0 : 1); }

    // C++ style implementation


    template <typename Type>
    inline PrintStream &operator<<(Type something){
        print(something);
        return *this;
    }

    // C style implementation

    std::size_t vprintf(const char *format, std::va_list args);
    std::size_t printf(const char *format, ...);
};

template <typename Integer>
std::size_t PrintStream::getIntegerDigits(Integer integer, std::uint8_t base) {
    std::size_t digits = 0;
    if (integer == 0)
        return 0;
    if (integer < 0)
        integer = -integer;
    while (integer != 0) {
        integer /= base;
        digits++;
    }
    return digits;
}

template <typename Floating>
inline std::size_t PrintStream::printFloat(Floating number, std::uint8_t digits) {
    size_t n = 0;
    if (std::isnan(number))
        return print("nan");
    if (std::isinf(number))
        return print("inf");
    if (number > std::numeric_limits<Floating>::max())
        return print("ovf"); // constant determined empirically
    if (number < std::numeric_limits<Floating>::min())
        return print("ovf"); // constant determined empirically

    // Handle negative numbers
    if (number < 0.0) {
        n += write('-');
        number = -number;
    }

    Floating rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
        rounding /= 10.0;

    number += rounding;

    unsigned long int_part = (unsigned long)number;
    Floating remainder = number - (Floating)int_part;
    n += print(int_part);

    if (digits > 0) {
        n += write('.');
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        unsigned int toPrint = (unsigned int)(remainder);
        n += write(getNumberCharInBase(toPrint, 10));
        remainder -= toPrint;
    }

    return n;
}

template <typename Floating>
inline std::size_t PrintStream::getFloatIntegerDigits(Floating floating) {
    std::size_t len = 0;
    if (floating < 0) {
        len++;
        floating = -floating;
    } else if (floating == 0) {
        return 1;
    }
    while (floating > 0) {
        floating /= 10;
        len++;
    }
    return len;
}

template <typename Interger>
std::size_t PrintStream::printInteger(Interger integer, std::uint8_t base) {
    std::size_t charsWritten = 0;
    std::size_t len = getIntegerDigits(integer, base);
    char buffer[len];
    if (integer < 0) {
        if (write('-') == false)
            return 0;
        charsWritten++;
        integer = -integer;
    }
    if (integer == 0) {
        charsWritten += write('0');
        return charsWritten;
    }
    for (std::size_t i = len; i > 0; i--) {
        buffer[i - 1] = getNumberCharInBase(integer % base, base);
        integer /= base;
    }
    for (std::size_t i = 0; i < len; i++) {
        if (write(buffer[i]) == false)
            return charsWritten;
        charsWritten++;
    }
    return charsWritten;
}

} // namespace ExLib
