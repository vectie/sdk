#include "highcontroller.h"
// #include "controllerbase.h"
#include "RotationTools.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <unistd.h>
using namespace org::eclipse::cyclonedds;

namespace legged {
DataBuffer<RobotMotorCmd::MotorCmdArray> motor_cmd_buffer_;
DataBuffer<std::array<MotorState, 24>> motor_state_buffer_;
DataBuffer<RobotControlCmd::ControlCmd> control_cmd_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;

HighController *HighController::instance = nullptr;
int readflag = 0;
bool HighController::init(ControlMode mode) {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        printf("cur path is %s\n", path.c_str());

        mode_ = WorkMode::DEFAULT;

        instance = this;

        RobotSetMode::SetMode cmode;
        cmode.mode(1);
        ddswrapper.publishModeData(cmode);

        // readbuf=hw.handread();
        // std::cout << "read data: ";
        // for (auto byte : readbuf) {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') <<
        //     (int)byte << " ";
        // }
        // std::cout << std::endl;
        // hw.write_mode(id_value,{0, 0, 0, 0, 0, 0});
        // if(flag == 0)
        // {
        //  hw.writereply({0xeb,0x90,0x01,0x0f,0x12,0x10,0x04,0x84,0x03,0x84,0x03,0x84,0x03,0x84,0x03,0xb0,0x04,0xa4,0x06,0xb0});
        // //  hw.write_angle(id_value, {900, 900, 900, 900, 1200, 1700});
        //    flag =1;
        // }else
        // {
        //   hw.writereply({0xeb,0x90,0x01,0x0f,0x12,0x10,0x04,0xb8,0x06,0xb8,0x06,0xb8,0x06,0xb8,0x06,0x46,0x05,0xa4,0x06,0x23});
        //   //hw.write_angle(id_value, {1720, 1720, 1720, 1720, 1350, 1700});
        //  // hw.write_angle(id_value, {1720, 1720, 1720, 1720, 1350, 1700});
        //    flag =0;
        // }
        // hw.writereply({0xeb,0x90,0x01,0x0f,0x12,0x16,0x04,0x70,0x17,0x70,0x17,0x70,0x17,0x70,0x17,0x70,0x17,0x70,0x17,0x66});

        // hw.write_force(id_value,{6000, 6000, 6000, 6000, 6000, 6000});
        // }
        // ddswrapper.subscribeHandReply([](const
        // PackageTransfer::TransferReply& ddsdata){
        //     std::vector<unsigned char> readbuf;
        //     auto& seq = ddsdata.reply_data();
        //     readbuf.assign(seq.begin(),seq.end());
        //     readflag =1;
        //     std::cout << "dds read data: ";
        //   for (auto byte : readbuf) {
        //       std::cout << std::hex << std::setw(2) << std::setfill('0') <<
        //       (int)byte << " ";
        //   }
        //   std::cout << std::endl;
        // });
        ddswrapper.subscribeRobotStatus(
            [](const RobotStatus::StatusData &ddsdata) {
                    std::array<MotorState, 24> data;
                    joydata remote_data;
                    NingImuData imudata;
                    static unsigned short premode = 200;
                    int i = 0;
                    for (const auto &state :
                         ddsdata.motorstatearray().motorstates()) {
                            data[i].pos = state.pos();
                            data[i].vel = state.vel();
                            data[i].tau = state.tau();
                            data[i].motor_id = state.motor_id();
                            data[i].error = state.error();
                            data[i].temperature = state.temperature();

                            // std::cout<< "motorstate  id: "<<
                            // motorstate_[i].motor_id<<";pos " <<
                            // motorstate_[i].pos << ";vel " <<
                            // motorstate_[i].vel
                            //<< ";tau "<<motorstate_[i].tau<<";error "
                            //<<motorstate_[i].error<<";temperature
                            //"<<motorstate_[i].temperature<<std::endl;
                            i++;
                    }
                    for (int i = 0; i < 4; i++) {
                            imudata.ori[i] = ddsdata.imudata().ori()[i];
                    }
                    for (int i = 0; i < 3; i++) {
                            imudata.angular_vel[i] =
                                ddsdata.imudata().angular_vel()[i];
                            imudata.linear_acc[i] =
                                ddsdata.imudata().linear_acc()[i];
                    }
                    for (int i = 0; i < 9; i++) {
                            imudata.ori_cov[i] = ddsdata.imudata().ori_cov()[i];
                            imudata.angular_vel_cov[i] =
                                ddsdata.imudata().angular_vel_cov()[i];
                            imudata.linear_acc_cov[i] =
                                ddsdata.imudata().linear_acc_cov()[i];
                    }
                    unsigned short curmode = ddsdata.workmode();
                    if (premode == 200) {
                            premode = curmode;
                    }
                    if (premode != curmode) {
                            premode = curmode;
                    }
                    // for(int i = 0;i<14;i++)
                    // {

                    //   if(ddsdata.joydata().button()[i] !=0)
                    //     printf("button %d =
                    //     %d\n",i,ddsdata.joydata().button()[i]);
                    // }
                    memcpy(remote_data.button, &ddsdata.joydata().button(),
                           sizeof(remote_data.button));
                    memcpy(remote_data.axes, &ddsdata.joydata().axes(),
                           sizeof(remote_data.axes));
                    // for(int i = 0;i<14;i++)
                    // {

                    //   if(remote_data.button[i] !=0)
                    //     printf("button %d = %d\n",i,remote_data.button[i]);
                    // }
                    HighController::Instance()->set_robotstatusdata(
                        data, imudata, remote_data);
            });
#if 0
    {
       
      printf("hand test\n");
      std::string id_value = "01";
      std::vector<unsigned char> writebuf={0xeb,0x90,0x01,0x0f,0x12,0x4c,0x04,0,0,0,0,0,0,0,0,0,0,0,0,0x72} ;
      PackageTransfer::TransferRequest request;
      auto& seq = request.packet_data();
      seq.assign(writebuf.begin(),writebuf.end());
      printf("publish request\n");
      ddswrapper.publishHandRequest(request);
      sleep(1);
      if(readflag == 1)
      {
        readflag=0;
        sleep(1);
      std::vector<unsigned char> data2={0xeb,0x90,0x01,0x0f,0x12,0x10,0x04,0x84,0x03,0x84,0x03,0x84,0x03,0x84,0x03,0xb0,0x04,0xa4,0x06,0xb0} ;
      seq.assign(data2.begin(),data2.end());
      ddswrapper.publishHandRequest(request);
      }
      usleep(500000);
      if(readflag == 1)
      {
        readflag=0;
        sleep(1);
      std::vector<unsigned char> data4={0xeb,0x90,0x01,0x0f,0x12,0x16,0x04,0x70,0x17,0x70,0x17,0x70,0x17,0x70,0x17,0x70,0x17,0x70,0x17,0x66};
      seq.assign(data4.begin(),data4.end());
      ddswrapper.publishHandRequest(request);
      }
      usleep(500000);
      if(readflag == 1)
      {
        readflag = 0;
      sleep(4);
      std::vector<unsigned char> data3={0xeb,0x90,0x01,0x0f,0x12,0x10,0x04,0xb8,0x06,0xb8,0x06,0xb8,0x06,0xb8,0x06,0x46,0x05,0xa4,0x06,0x23};
      seq.assign(data3.begin(),data3.end());
      ddswrapper.publishHandRequest(request);

      }
    }
#endif
        send_thread_ = std::thread(&HighController::send_thread_func, this);
        process_thread_ =
            std::thread(&HighController::process_thread_func, this);
        sched_param ddssched{.sched_priority = 98};
        if (pthread_setschedparam(process_thread_.native_handle(), SCHED_FIFO,
                                  &ddssched) != 0) {
                printf(" failed to set threads priority\n");
        }
        if (pthread_setschedparam(send_thread_.native_handle(), SCHED_FIFO,
                                  &ddssched) != 0) {
                printf(" failed to set threads priority\n");
        }
        return true;
}
void HighController::set_imudata(NingImuData data) {
        for (int i = 0; i < 4; i++) {
                imu_data_.ori[i] = data.ori[i];
        }
        for (int i = 0; i < 3; i++) {
                imu_data_.angular_vel[i] = data.angular_vel[i];
                imu_data_.linear_acc[i] = data.linear_acc[i];
        }
        for (int i = 0; i < 9; i++) {
                imu_data_.ori_cov[i] = data.ori_cov[i];
                imu_data_.angular_vel_cov[i] = data.angular_vel_cov[i];
                imu_data_.linear_acc_cov[i] = data.linear_acc_cov[i];
        }
}
void HighController::set_joydata(joydata data) {
        memcpy(remote_data_.button, &data.button, sizeof(data.button));
        memcpy(remote_data_.axes, &data.axes, sizeof(data.axes));
}
void HighController::set_robotstatusdata(std::array<MotorState, 24> data,
                                         NingImuData imudata,
                                         joydata joy_data) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
}

void HighController::send_thread_func() {
        while (1) {
                const std::shared_ptr<const RobotMotorCmd::MotorCmdArray> mc =
                    motor_cmd_buffer_.GetData();
                if (mc) {
                        RobotMotorCmd::MotorCmdArray cmdarray;
                        cmdarray.motorcmds().resize(24);
                        for (int i = 0; i < 24; i++) {
                                auto &cmd = cmdarray.motorcmds()[i];

                                cmd.pos() = mc->motorcmds().at(i).pos();
                                cmd.vel() = mc->motorcmds().at(i).vel();
                                cmd.tau() = mc->motorcmds().at(i).tau();
                                cmd.kp() = mc->motorcmds().at(i).kp();
                                cmd.kd() = mc->motorcmds().at(i).kd();
                                cmd.motor_id() =
                                    mc->motorcmds().at(i).motor_id();
                        }
                        // cmdarray.controlcmd().axes()[0] =
                        // mc->controlcmd().axes().at(0);
                        // cmdarray.controlcmd().axes()[1] =
                        // mc->controlcmd().axes().at(1);
                        // cmdarray.controlcmd().action() =
                        // mc->controlcmd().action();
                        auto now = Clock::now();
                        long long timestamp = std::chrono::duration_cast<
                                                  std::chrono::microseconds>(
                                                  now.time_since_epoch())
                                                  .count();
                        cmdarray.timestamp() = timestamp;
                        ddswrapper.publishMotorCmdData(cmdarray);
                }

                std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
}
void HighController::process_thread_func() {
        while (1) {
                auto start_time = std::chrono::steady_clock::now();
                process();
                auto end_time = std::chrono::steady_clock::now();
                auto elapsed =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time);
                std::this_thread::sleep_for(std::chrono::microseconds(2000) -
                                            elapsed);
        }
}
const NingImuData HighController::get_imu_data() { return imu_data_; }
const joydata HighController::get_jsdata() { return remote_data_; }

void HighController::set_axes(double ver, double hor, int action,
                              uint16_t index) {

        RobotControlCmd::ControlCmd controlcmd;

        auto now = Clock::now();
        long long timestamp =
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch())
                .count();
        int diff = timestamp - statetimestamp;

        controlcmd.axes()[0] = hor;
        controlcmd.axes()[1] = ver;
        controlcmd.action() = action;
        if (action == 9 || action == 11) {
                controlcmd.data() = index;
        }

        control_cmd_buffer_.SetData(controlcmd);
}
const std::array<MotorState, 24> HighController::get_joint_state() {
        std::array<MotorState, 24> motorstate;
        const std::shared_ptr<const std::array<MotorState, 24>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                for (int i = 0; i < 24; i++) {
                        motorstate[i].pos = ms->at(i).pos;
                        motorstate[i].vel = ms->at(i).vel;
                        motorstate[i].tau = ms->at(i).tau;
                        motorstate[i].motor_id = ms->at(i).motor_id;
                        motorstate[i].error = ms->at(i).error;
                        motorstate[i].temperature = ms->at(i).temperature;
                }
        }
        return motorstate;
}

uint16_t fileindex = 0;

void HighController::process() {

        static int keyflag[14];
        joydata remote_data;
        static char key_updown[14], key_inuse[14];
        Command cmd;
        auto now = Clock::now();
        long starttimestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();
        const std::shared_ptr<const joydata> mc = joy_buffer_.GetData();
        if (mc) {
                for (int i = 0; i < 14; i++) {
                        remote_data.button[i] = mc->button[i];
                }
                remote_data.axes[0] = mc->axes[0];
                remote_data.axes[1] = mc->axes[1];
        }

        for (int i = 0; i < 14; i++) {
                if (remote_data.button[i] == 0) {
                        key_updown[i] = 0;
                        key_inuse[i] = 0;
                } else if (remote_data.button[i] == 1) {
                        key_updown[i] = 1;
                }
        }
        ControlCmd action = ControlCmd::DEFAULT;
        cmd.x = remote_data.axes[1];
        cmd.y = 0;
        cmd.yaw = remote_data.axes[0];
        if (key_updown[Key2] == 0 && key_updown[Key5] == 1 &&
            (key_inuse[Key5] == 0)) {
                action = ControlCmd::STARTTEACH;
                key_inuse[Key5] = 1;
                printf("STARTTEACH \n");
        } else if ((key_updown[Key6] == 1) && key_updown[Key2] == 0 &&
                   (key_inuse[Key6] == 0) && key_updown[Key1] == 0) {
                action = ControlCmd::SWING;
                key_inuse[Key6] = 1;
                printf("SWING \r\n");
        } else if ((key_updown[Key7] == 1) && key_updown[Key2] == 0 &&
                   (key_inuse[Key7] == 0) && key_updown[Key1] == 0) {
                action = ControlCmd::SHAKE;
                key_inuse[Key7] = 1;
                printf("SHAKE \r\n");
        } else if ((key_updown[Key8] == 1) && key_updown[Key2] == 0 &&
                   (key_inuse[Key8] == 0) && key_updown[Key1] == 0) {
                action = ControlCmd::CHEER;
                key_inuse[Key8] = 1;
                printf("CHEER \r\n");
        } else if ((key_updown[Key9] == 1) && (key_inuse[Key9] == 0)) {
                key_inuse[Key9] = 1;
                action = ControlCmd::START;
        } else if (key_updown[Key10] == 1 && (key_inuse[Key10] == 0)) {
                action = ControlCmd::SWITCH;
                key_inuse[Key10] = 1;
                printf("SWITCH \n");
        } else if (key_updown[Key2] == 1 && key_updown[Key5] == 1 &&
                   (key_inuse[Key5] == 0)) {
                action = ControlCmd::WALK;
                key_inuse[Key5] = 1;
                printf("WALK \n");
        } else if (key_updown[Key1] == 1 && key_updown[Key6] == 1 &&
                   (key_inuse[Key6] == 0)) {
                action = ControlCmd::SAVETEACH;
                key_inuse[Key6] = 1;
                fileindex++;
                printf("SAVETEACH \n");
        } else if (key_updown[Key1] == 1 && key_updown[Key7] == 1 &&
                   (key_inuse[Key7] == 0)) {
                action = ControlCmd::ENDTEACH;
                key_inuse[Key7] = 1;
                printf("ENDTEACH \n");
        } else if (key_updown[Key1] == 1 && key_updown[Key8] == 1 &&
                   (key_inuse[Key8] == 0)) {
                action = ControlCmd::PLAYTEACH;
                key_inuse[Key8] = 1;
                fileindex = 1;
                printf("PLAYTEACH \n");
        } else if(key_updown[Key2] == 1 && key_updown[Key6] == 1) {
                action = ControlCmd::RUN;
                key_inuse[Key6] = 1;
	}
        RobotControlCmd::ControlCmd controlcmd;
        controlcmd.axes()[0] = cmd.yaw;
        controlcmd.axes()[1] = cmd.x;
        controlcmd.action() = (int)action;
        controlcmd.data() = fileindex;
        ddswrapper.publishControlCmdData(controlcmd);
}

} // namespace legged

int main() {

        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        legged::HighController highcontroller;
        highcontroller.init(legged::ControlMode::HIGHMODE);

        while (1) {
                usleep(10);
        }
        return 0;
}
