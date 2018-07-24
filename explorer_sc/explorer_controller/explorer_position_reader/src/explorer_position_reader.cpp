#include "explorer_position_reader/explorer_position_reader.h"
positionReader::positionReader() {}
positionReader::~positionReader() {}
bool positionReader::init(explorer_position_reader::PosionReaderInterface *posion_interface/*form hardware*/, ros::NodeHandle &, ros::NodeHandle &) {}
// start 与 stop 函数采用 Controller 默认的函数,即置空

// 更新函数,每次机械臂的位置数据更新都在这个函数里进行
void positionReader::update(const ros::Time &, const ros::Duration &) {}

