#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量
    
    // Initialize gripper flags
    _has_gripper = false;
    
    // Parse hardware parameters for gripper configuration
    if (info_.hardware_parameters.find("gripper_company") != info_.hardware_parameters.end()) {
        _has_gripper = true;
        _gripper_company = std::stoi(info_.hardware_parameters.at("gripper_company"));
        _gripper_device = std::stoi(info_.hardware_parameters.at("gripper_device"));
        _gripper_softversion = std::stoi(info_.hardware_parameters.at("gripper_softversion"));
        _gripper_bus = std::stoi(info_.hardware_parameters.at("gripper_bus"));
        _gripper_index = std::stoi(info_.hardware_parameters.at("gripper_index"));
        _gripper_max_time = std::stoi(info_.hardware_parameters.at("gripper_max_time"));
        
        // Get control parameters with defaults
        _gripper_position_tolerance = info_.hardware_parameters.find("gripper_position_tolerance") != info_.hardware_parameters.end() 
            ? std::stod(info_.hardware_parameters.at("gripper_position_tolerance")) : 0.001;
        _gripper_max_velocity = info_.hardware_parameters.find("gripper_max_velocity") != info_.hardware_parameters.end() 
            ? std::stod(info_.hardware_parameters.at("gripper_max_velocity")) : 50.0;
        _gripper_max_effort = info_.hardware_parameters.find("gripper_max_effort") != info_.hardware_parameters.end() 
            ? std::stod(info_.hardware_parameters.at("gripper_max_effort")) : 50.0;
            
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper parameters detected and configured");
    }

    // Process joints (both arm and gripper)
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        // Check if this is a gripper joint
        if (joint.name.find("finger") != std::string::npos) {
            // This is a gripper joint
            if (!_has_gripper) {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                          "Found gripper joint '%s' but no gripper parameters configured", joint.name.c_str());
            } else {
                // Only add the left finger joint (primary joint), ignore mimic joints
                if (joint.name == "left_finger_joint") {
                    _gripper_joint_names.push_back(joint.name);
                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                              "Added gripper joint: %s", joint.name.c_str());
                }
            }
            
            // Validate gripper joint interfaces
            // Only validate command interfaces for joints that actually have them
            // (right_finger_joint is a mimic joint with only state interfaces)
            if (joint.command_interfaces.size() > 1) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                            "Gripper joint '%s' has %zu command interfaces found. 0 or 1 expected.", joint.name.c_str(),
                            joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Only validate command interface type if the joint has command interfaces
            if (joint.command_interfaces.size() == 1 && 
                joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                       "Gripper joint '%s' have %s command interface. '%s' expected.",
                       joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 3) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), 
                            "Gripper joint '%s' has %zu state interfaces. 3 expected.",
                            joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else {
            // This is an arm joint - validate as before
            if (joint.command_interfaces.size() != 1) {//开放servoJ
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                            joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                       "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                       joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            //关节状态部分
            if (joint.state_interfaces.size() != 1) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 1 expected.",
                            joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                            "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
    }
    
    // Initialize gripper vectors if gripper is present
    if (_has_gripper && !_gripper_joint_names.empty()) {
        size_t gripper_joints_count = _gripper_joint_names.size();
        _gripper_position_command.resize(gripper_joints_count, 0.0);
        _gripper_velocity_command.resize(gripper_joints_count, 0.0);
        _gripper_effort_command.resize(gripper_joints_count, 0.0);
        _gripper_position_state.resize(gripper_joints_count, 0.0);
        _gripper_velocity_state.resize(gripper_joints_count, 0.0);
        _gripper_effort_state.resize(gripper_joints_count, 0.0);
        
        // Initialize gripper status
        _gripper_active = false;
        _gripper_position = 0;
        _gripper_current = 0;
        _gripper_voltage = 0;
        _gripper_temperature = 0;
        _gripper_speed = 0;
        _gripper_fault = 0;
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), 
                   "Initialized %zu gripper joints", gripper_joints_count);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    // Check if this is a gripper joint
    if (info_.joints[i].name.find("finger") != std::string::npos) {
      // For gripper joints, we need to handle both primary and mimic joints
      if (info_.joints[i].name == "left_finger_joint") {
        // Primary gripper joint - use gripper joint index
        auto it = std::find(_gripper_joint_names.begin(), _gripper_joint_names.end(), info_.joints[i].name);
        if (it != _gripper_joint_names.end()) {
          size_t gripper_idx = std::distance(_gripper_joint_names.begin(), it);
          state_interfaces.emplace_back(hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_gripper_position_state[gripper_idx]));
          state_interfaces.emplace_back(hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_gripper_velocity_state[gripper_idx]));
          state_interfaces.emplace_back(hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_gripper_effort_state[gripper_idx]));
        }
      } else if (info_.joints[i].name == "right_finger_joint") {
        // Mimic joint - use the same state as the primary joint (index 0)
        if (!_gripper_joint_names.empty()) {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_gripper_position_state[0]));
          state_interfaces.emplace_back(hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_gripper_velocity_state[0]));
          state_interfaces.emplace_back(hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_gripper_effort_state[0]));
        }
      }
    } else {
      // This is an arm joint - map to arm joint index
      // Extract joint number from joint name (e.g., "j1" -> index 0)
      if (info_.joints[i].name.size() >= 2 && info_.joints[i].name[0] == 'j') {
        int joint_idx = std::stoi(info_.joints[i].name.substr(1)) - 1; // j1 -> index 0
        if (joint_idx >= 0 && joint_idx < 6) {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[joint_idx]));
        }
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // Check if this is a gripper joint
    if (info_.joints[i].name.find("finger") != std::string::npos) {
      // Find the index in gripper joint names
      auto it = std::find(_gripper_joint_names.begin(), _gripper_joint_names.end(), info_.joints[i].name);
      if (it != _gripper_joint_names.end()) {
        size_t gripper_idx = std::distance(_gripper_joint_names.begin(), it);
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_gripper_position_command[gripper_idx]));
      }
    } else {
      // This is an arm joint - map to arm joint index
      if (info_.joints[i].name.size() >= 2 && info_.joints[i].name[0] == 'j') {
        int joint_idx = std::stoi(info_.joints[i].name.substr(1)) - 1; // j1 -> index 0
        if (joint_idx >= 0 && joint_idx < 6) {
          command_interfaces.emplace_back(hardware_interface::CommandInterface(
              info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[joint_idx]));
        }
      }
    }
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
    //做变量的初始化工作
    _ptr_robot = std::make_unique<FRRobot>();//创建机器人实例
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    errno_t returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
    rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
    if(returncode != 0){
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接失败！请检查端口时候被占用");
        return hardware_interface::CallbackReturn::ERROR;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接成功！");
    }
    
    // Configure gripper if present
    if (_has_gripper && !_gripper_joint_names.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Configuring gripper...");
        
        if (!configure_gripper()) {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to configure gripper");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (!activate_gripper(true)) {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to activate gripper");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Read initial gripper state
        if (!read_gripper_status()) {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read initial gripper status");
        }
        
        // Set initial gripper command positions
        for (size_t i = 0; i < _gripper_position_command.size(); i++) {
            _gripper_position_command[i] = _gripper_position_state[i];
        }
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper configured and activated successfully");
    }
    
    //做第一步的工作，读取当前状态数据
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
    if(returncode == 0){
        for(int j=0;j<6;j++){
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);    
        
        _last_read_time = std::chrono::steady_clock::now();
        _last_write_time = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    
    // Deactivate gripper if present
    if (_has_gripper && _gripper_active) {
        activate_gripper(false);
    }
    
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;//注意单位转换，moveit统一用弧度
            //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
        }
    }else{
        return hardware_interface::return_type::ERROR;
    }
    
    // Read gripper status if present
    if (_has_gripper && !_gripper_joint_names.empty()) {
        if (read_gripper_status()) {
            // Update gripper position, velocity, and effort
            for (size_t i = 0; i < _gripper_position_state.size(); i++) {
                _gripper_position_state[i] = gripper_units_to_meters(_gripper_position);
                _gripper_velocity_state[i] = static_cast<double>(_gripper_speed) / 100.0; // Convert to m/s
                _gripper_effort_state[i] = static_cast<double>(_gripper_current) / 100.0;  // Convert to appropriate units
            }
        }
    }
    
    _last_read_time = std::chrono::steady_clock::now();
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _jnt_position_command[j]/M_PI*180; //注意单位转换
        }
        //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
            cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.008,0,0);
        if(returncode != 0){
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ指令下发错误,错误码:%d",returncode);
        }
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        //_ptr_robot->write(_jnt_torque_command);//注意单位转换
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }
    
    // Handle gripper commands if present
    if (_has_gripper && !_gripper_joint_names.empty()) {
        for (size_t i = 0; i < _gripper_position_command.size(); i++) {
            if (std::isnan(_gripper_position_command[i])) {
                continue;
            }

            double position_diff = std::abs(_gripper_position_command[i] - _gripper_position_state[i]);
            
            if (position_diff > _gripper_position_tolerance) {
                // Send command to gripper
                if (!move_gripper_to_position(_gripper_position_command[i], _gripper_max_velocity, _gripper_max_effort)) {
                    RCLCPP_ERROR(
                      rclcpp::get_logger("FairinoHardwareInterface"), 
                      "Failed to move gripper to position %f", _gripper_position_command[i]);
                    return hardware_interface::return_type::ERROR;
                }
                
                RCLCPP_DEBUG(
                  rclcpp::get_logger("FairinoHardwareInterface"),
                  "Commanding gripper to position: %f", _gripper_position_command[i]);
            }
        }
    }
    
    _last_write_time = std::chrono::steady_clock::now();
 
    return hardware_interface::return_type::OK;
}

// Gripper helper methods implementation
bool FairinoHardwareInterface::configure_gripper()
{
  try
  {
    // Set gripper configuration
    _ptr_robot->SetGripperConfig(_gripper_company, _gripper_device, _gripper_softversion, _gripper_bus);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Verify configuration
    int company, device, softversion, bus;
    _ptr_robot->GetGripperConfig(&company, &device, &softversion, &bus);

    RCLCPP_INFO(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Gripper configured - Company: %d, Device: %d, Version: %d, Bus: %d",
      company, device, softversion, bus);

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Exception during gripper configuration: %s", e.what());
    return false;
  }
}

bool FairinoHardwareInterface::activate_gripper(bool activate)
{
  try
  {
    int act = activate ? 1 : 0;
    _ptr_robot->ActGripper(_gripper_index, act);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    _gripper_active = activate;
    RCLCPP_INFO(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Gripper %s", activate ? "activated" : "deactivated");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Exception during gripper activation: %s", e.what());
    return false;
  }
}

bool FairinoHardwareInterface::move_gripper_to_position(
  double position, double velocity, double effort)
{
  try
  {
    uint8_t gripper_pos = meters_to_gripper_units(position);
    uint8_t block = 0; // Non-blocking
    int type = 0; // Standard gripper type (not rotational)
    double rotNum = 0.0; // No rotation
    int rotVel = 0; // No rotation velocity
    int rotTorque = 0; // No rotation torque

    _ptr_robot->MoveGripper(
      _gripper_index,
      static_cast<int>(gripper_pos),
      static_cast<int>(velocity),
      static_cast<int>(effort),
      _gripper_max_time,
      block,
      type,
      rotNum,
      rotVel,
      rotTorque);

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Exception during gripper movement: %s", e.what());
    return false;
  }
}

bool FairinoHardwareInterface::read_gripper_status()
{
  try
  {
    uint16_t fault;
    uint16_t active_status;

    // Get current position
    _ptr_robot->GetGripperCurPosition(&fault, &_gripper_position);
    _gripper_fault = fault;

    // Get current
    _ptr_robot->GetGripperCurCurrent(&fault, &_gripper_current);

    // Get voltage
    _ptr_robot->GetGripperVoltage(&fault, &_gripper_voltage);

    // Get temperature
    _ptr_robot->GetGripperTemp(&fault, &_gripper_temperature);

    // Get speed
    _ptr_robot->GetGripperCurSpeed(&fault, &_gripper_speed);

    // Get active status
    _ptr_robot->GetGripperActivateStatus(&fault, &active_status);
    _gripper_active = (active_status == 1);

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Exception during status read: %s", e.what());
    return false;
  }
}

double FairinoHardwareInterface::gripper_units_to_meters(uint8_t gripper_units)
{
  // Convert gripper units (0-100) to meters (0-0.1m for example)
  // Adjust this conversion based on your gripper's actual range
  return static_cast<double>(gripper_units) / 1000.0; // 100 units = 0.1 meters
}

uint8_t FairinoHardwareInterface::meters_to_gripper_units(double meters)
{
  // Convert meters to gripper units (0-100)
  // Clamp to valid range
  double units = meters * 1000.0;
  units = std::max(0.0, std::min(100.0, units));
  return static_cast<uint8_t>(units);
}

bool FairinoHardwareInterface::is_gripper_motion_done()
{
  try
  {
    uint16_t fault;
    uint8_t status;
    _ptr_robot->GetGripperMotionDone(&fault, &status);
    return (status == 1);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("FairinoHardwareInterface"),
      "Exception checking motion status: %s", e.what());
    return false;
  }
}

}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
