#ifndef EXPLORER_POSITION_READER_INTERFACE_HPP
#define EXPLORER_POSITION_READER_INTERFACE_HPP

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace explorer_interface {

class PosionReaderHandle {
public:
    PosionReaderHandle() : name_(), pos_(0) {}

    PosionReaderHandle(const std::string &name, double *pos)
        : name_(name), pos_(pos) {}

    std::string getName() const {
        return name_;
    }
    double getPosition()  const {
        return *pos_;
    }
    void setPosition(double pos_new) {
        *pos_ = pos_new;
    }

private:
    std::string name_;
    double *pos_;
};

class PosionReaderInterface : public hardware_interface::HardwareResourceManager <PosionReaderHandle> {};

}

#endif // EXPLORER_POSITION_READER_INTERFACE_HPP


