#include "ExLib_ReadStreamBuffered.hpp"

namespace ExLib {
ReadStreamBuffered::ReadStreamBuffered(BufferFIFO<char> &buffer)
    : buffer(buffer) {
}
bool ReadStreamBuffered::read(char &ch) {
    if(buffer.avaliable()==false)
        return false;
    buffer.read(ch);
    return true;
}
bool ReadStreamBuffered::peek(char &ch) {
    return false;
}
std::size_t ReadStreamBuffered::avaliableForRead(void) {
    return buffer.avaliable();
}
} // namespace ExLib