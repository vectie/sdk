#include "lowcontroller.h"
#include "RotationTools.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <chrono>
#include <common.h>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <unistd.h>

namespace legged {
// Controllerbase controllerbase;
DataBuffer<std::array<MotorCmd, 24>> motor_cmd_buffer_;
DataBuffer<std::array<MotorState, 24>> motor_state_buffer_;
DataBuffer<RobotControlCmd::ControlCmd> control_cmd_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;

LowController *LowController::instance = nullptr;
std::vector<scalar_t> currentJointAngles_;
bool LowController::init(ControlMode mode) {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        // printf("cur path is %s\n",path.c_str());
        YAML::Node acconfig = YAML::LoadFile(path + "/config/ning_user.yaml");
        mode_ = WorkMode::DEFAULT;
        standDuration = 1000;
        standPercent = 0;
        lieJointAngles_.resize(24);
        standJointAngles_.resize(24);
        currentJointAngles_.resize(24);
        auto &LieState = liejointState_;
        auto &StandState = standjointState_;
        lieJointAngles_ << LieState.leg_l1_joint, LieState.leg_r1_joint,
            LieState.waist_1_joint, LieState.leg_l2_joint,
            LieState.leg_r2_joint, LieState.arm_l1_joint, LieState.arm_r1_joint,
            LieState.leg_l3_joint, LieState.leg_r3_joint, LieState.arm_l2_joint,
            LieState.arm_r2_joint, LieState.leg_l4_joint, LieState.leg_r4_joint,
            LieState.arm_l3_joint, LieState.arm_r3_joint, LieState.leg_l5_joint,
            LieState.leg_r5_joint, LieState.arm_l4_joint, LieState.arm_r4_joint,
            LieState.leg_l6_joint, LieState.leg_r6_joint, LieState.arm_l5_joint,
            LieState.arm_r5_joint, LieState.waist_2_joint;

        standJointAngles_ << StandState.leg_l1_joint, StandState.leg_r1_joint,
            StandState.waist_1_joint, StandState.leg_l2_joint,
            StandState.leg_r2_joint, StandState.arm_l1_joint,
            StandState.arm_r1_joint, StandState.leg_l3_joint,
            StandState.leg_r3_joint, StandState.arm_l2_joint,
            StandState.arm_r2_joint, StandState.leg_l4_joint,
            StandState.leg_r4_joint, StandState.arm_l3_joint,
            StandState.arm_r3_joint, StandState.leg_l5_joint,
            StandState.leg_r5_joint, StandState.arm_l4_joint,
            StandState.arm_r4_joint, StandState.leg_l6_joint,
            StandState.leg_r6_joint, StandState.arm_l5_joint,
            StandState.arm_r5_joint, StandState.waist_2_joint;
        instance = this;

        RobotSetMode::SetMode cmode;

        if (mode == ControlMode::LOWMODE) {
                cmode.mode(2);
        }

        ddswrapper.publishModeData(cmode);
        ddswrapper.subscribeRobotStatus(
            [](const RobotStatus::StatusData &ddsdata) {
                    std::array<MotorState, 24> data;
                    joydata remote_data;
                    NingImuData imudata;
                    int i = 0;
                    for (const auto &state :
                         ddsdata.motorstatearray().motorstates()) {
                            data[i].pos = state.pos();
                            data[i].vel = state.vel();
                            data[i].tau = state.tau();
                            data[i].motor_id = state.motor_id();
                            data[i].error = state.error();
                            data[i].temperature = state.temperature();
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
                    memcpy(remote_data.button, &ddsdata.joydata().button(),
                           sizeof(remote_data.button));
                    memcpy(remote_data.axes, &ddsdata.joydata().axes(),
                           sizeof(remote_data.axes));

                    LowController::Instance()->set_robotstatusdata(
                        data, imudata, remote_data);
            });

        process_thread_ =
            std::thread(&LowController::process_thread_func, this);
        send_thread_ = std::thread(&LowController::send_thread_func, this);
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

void LowController::set_robotstatusdata(std::array<MotorState, 24> data,
                                        NingImuData imudata, joydata joy_data) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
}

void LowController::send_thread_func() {
        while (1) {
                const std::shared_ptr<const std::array<MotorCmd, 24>> mc =
                    motor_cmd_buffer_.GetData();
                if (mc) {
                        RobotMotorCmd::MotorCmdArray cmdarray;
                        cmdarray.motorcmds().resize(24);
                        for (int i = 0; i < 24; i++) {
                                auto &cmd = cmdarray.motorcmds()[i];
                                cmd.pos() = mc->at(i).pos;
                                cmd.vel() = mc->at(i).vel;
                                cmd.tau() = mc->at(i).tau;
                                cmd.kp() = mc->at(i).kp;
                                cmd.kd() = mc->at(i).kd;
                                cmd.motor_id() = mc->at(i).motor_id;
                        }

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

void LowController::process_thread_func() {
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

void LowController::set_joint(std::array<MotorCmd, 24> motorcmd) {
        motor_cmd_buffer_.SetData(motorcmd);
}
const std::array<MotorState, 24> LowController::get_joint_state() {
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

int LowController::getJointsIndex(std::string jointname) {
        int index = 0;
        if (jointname == "arm_l1_joint") {
                index = 0;
        } else if (jointname == "arm_l2_joint") {
                index = 1;
        } else if (jointname == "arm_l3_joint") {
                index = 2;
        } else if (jointname == "arm_l4_joint") {
                index = 3;
        } else if (jointname == "arm_l5_joint") {
                index = 4;
        } else if (jointname == "leg_l1_joint") {
                index = 5;
        } else if (jointname == "leg_l2_joint") {
                index = 6;
        } else if (jointname == "leg_l3_joint") {
                index = 7;
        } else if (jointname == "leg_l4_joint") {
                index = 8;
        } else if (jointname == "leg_l5_joint") {
                index = 9;
        } else if (jointname == "leg_l6_joint") {
                index = 10;
        } else if (jointname == "arm_r1_joint") {
                index = 11;
        } else if (jointname == "arm_r2_joint") {
                index = 12;
        } else if (jointname == "arm_r3_joint") {
                index = 13;
        } else if (jointname == "arm_r4_joint") {
                index = 14;
        } else if (jointname == "arm_r5_joint") {
                index = 15;
        } else if (jointname == "leg_r1_joint") {
                index = 16;
        } else if (jointname == "leg_r2_joint") {
                index = 17;
        } else if (jointname == "leg_r3_joint") {
                index = 18;
        } else if (jointname == "leg_r4_joint") {
                index = 19;
        } else if (jointname == "leg_r5_joint") {
                index = 20;
        } else if (jointname == "leg_r6_joint") {
                index = 21;
        } else if (jointname == "waist_1_joint") {
                index = 22;
        } else if (jointname == "waist_2_joint") {
                index = 23;
        }
        return index;
}

void LowController::setparameter(Command &cmd, bool *isfirst) {
        isfirstRecObs_ = isfirst;
        isfirstCompAct_ = *isfirstRecObs_;
        command_[0] = cmd.x;
        command_[1] = cmd.y;
        command_[2] = cmd.yaw;
}

bool LowController::updateStateEstimation() {
        vector_t jointPosnoarm(24), jointVelnoarm(24), jointTornoarm(24);
        quaternion_t quat;
        vector3_t angularVel, linearAccel;
        static int num = 0;
        std::array<MotorState, 24> joint_state;
        const std::shared_ptr<const std::array<MotorState, 24>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                for (int i = 0; i < 24; i++) {
                        joint_state[i].pos = ms->at(i).pos;
                        joint_state[i].vel = ms->at(i).vel;
                        joint_state[i].tau = ms->at(i).tau;
                }
        }
        NingImuData imudata;
        std::chrono::microseconds now =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        int j = 0;
        for (size_t i = 0; i < actuatedDofNum_; ++i) {
                int index = getJointsIndex(walkjointNames[i]);
                {
                        jointPosnoarm(i) = joint_state[index].pos;
                        jointVelnoarm(i) = joint_state[index].vel;
                        jointTornoarm(i) = joint_state[index].tau;
                        if (abs(jointPosnoarm(i)) == 12.5) {
                                // LogError(" {} disconnect",jointNames[i]);
                                // return false;
                        }
                }
        }

        const std::shared_ptr<const NingImuData> idata = imu_buffer_.GetData();
        if (idata) {
                for (int i = 0; i < 4; i++) {
                        imudata.ori[i] = (*idata).ori[i];
                }
                for (int i = 0; i < 3; i++) {
                        imudata.angular_vel[i] = (*idata).angular_vel[i];
                        imudata.linear_acc[i] = (*idata).linear_acc[i];
                }
                for (int i = 0; i < 9; i++) {
                        imudata.ori_cov[i] = (*idata).ori_cov[i];
                        imudata.angular_vel_cov[i] =
                            (*idata).angular_vel_cov[i];
                        imudata.linear_acc_cov[i] = (*idata).linear_acc_cov[i];
                }

                // imudata = get_imu_data();
                for (size_t i = 0; i < 4; ++i) {
                        quat.coeffs()(i) = imudata.ori[i];
                }
                for (size_t i = 0; i < 3; ++i) {
                        angularVel(i) = imudata.angular_vel[i];
                        linearAccel(i) = imudata.linear_acc[i];
                        // printf("imudata.angular_vel %f  imudata.linear_acc %f
                        // i
                        // %d\n",imudata.angular_vel[i],imudata.linear_acc[i],i);
                }

                propri_.jointPos = jointPosnoarm;
                propri_.jointVel = jointVelnoarm;
                propri_.baseAngVel = angularVel;

                vector3_t gravityVector(0, 0, -1);
                vector3_t zyx = quatToZyx(quat);
                matrix_t inverseRot =
                    getRotationMatrixFromZyxEulerAngles(zyx).inverse();
                propri_.projectedGravity = inverseRot * gravityVector;
                propri_.baseEulerXyz = quatToXyz(quat);
        }
        double seconds = now.count();
        phase_ = seconds / 1000000.0;

        return true;
}

void LowController::handleDefautMode() {
        // MotorCmd  motorcmd;
        std::array<MotorCmd, 24> motorcmd;
        for (int j = 0; j < 24; j++) {
                motorcmd[j].kd = 0.1;
                motorcmd[j].pos = 0;
                motorcmd[j].kp = 0;
                motorcmd[j].motor_id = j;
                motorcmd[j].vel = 0;
                motorcmd[j].tau = 0;
                // controllerbase.set_joint(motorcmd);
        }

        set_joint(motorcmd);
}

void LowController::handleStandMode() {
        // MotorCmd motorcmd;
        std::array<MotorCmd, 24> motorcmd;
        if (standPercent <= 1) {
                for (int j = 0; j < 24; j++) {
                        scalar_t pos_des =
                            currentJointAngles_[j] * (1 - standPercent) +
                            standJointAngles_[j] * standPercent;
                        int index = getJointsIndex(jointNames[j]);
                        if (j == 23) {
                                motorcmd[j].kd = 5;
                                motorcmd[j].motor_id = index;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;
                                // hw.set_joint(motorcmd);
                        } else if (j == 15 || j == 16 || j == 19 || j == 20) {

                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 50;
                                motorcmd[j].kd = 1;
                                motorcmd[j].motor_id = index;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;
                                // hw.set_joint(motorcmd);
                        } else {
                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 50;
                                motorcmd[j].kd = 5;
                                motorcmd[j].motor_id = index;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;
                                // hw.set_joint(motorcmd);
                        }
                }
                set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, scalar_t(1));
        }
}

void LowController::handleLieMode() {
        // MotorCmd motorcmd;
        std::array<MotorCmd, 24> motorcmd;

        if (standPercent <= 1) {
                for (int j = 0; j < 24; j++) {
                        scalar_t pos_des =
                            currentJointAngles_[j] * (1 - standPercent) +
                            lieJointAngles_[j] * standPercent;
                        int index = getJointsIndex(jointNames[j]);
                        if (j == 23) {
                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 200;
                                motorcmd[j].kd = 5;
                                motorcmd[j].motor_id = index;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;
                                // hw.set_joint(motorcmd);
                        } else if (j == 15 || j == 16 || j == 19 || j == 20) {
                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 50;
                                motorcmd[j].kd = 1;
                                motorcmd[j].motor_id = index;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;
                                // hw.set_joint(motorcmd);
                        } else {
                                motorcmd[j].pos = pos_des;
                                motorcmd[j].kp = 70;
                                motorcmd[j].kd = 5;
                                motorcmd[j].motor_id = index;
                                motorcmd[j].vel = 0;
                                motorcmd[j].tau = 0;
                                // hw.set_joint(motorcmd);
                        }
                }
                set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, double(1));
        }
}

bool LowController::handleUserMode() {
        if (updateStateEstimation() == false)
                return false;
        if (count % robotconfig.controlCfg.decimation == 0) {
                count = 0;
                computeObservation();
                computeActions();

                // limit action range
                scalar_t actionMin = -robotconfig.clipActions;
                scalar_t actionMax = robotconfig.clipActions;
                std::transform(
                    actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](scalar_t x) {
                            return std::max(actionMin, std::min(actionMax, x));
                    });
        }
        // set action
        int j = 0;
        // MotorCmd motorcmd;
        std::vector<int> jointIndicesLeg = {
            0, 1, 3, 4, 6, 7, 10, 11, 14, 15, 18, 19,
        };
        std::vector<int> jointIndicesArm = {2,  5,  8,  9,  12, 13,
                                            16, 17, 20, 21, 22, 23};
        std::array<MotorCmd, 24> motorcmd;
        for (int i = 0; i < jointIndicesArm.size(); i++) {
                double kp = 150;
                double kd = 5;
                double pos = 0;
                int jointIdx = jointIndicesArm[i];
                if (i < 2) {
                        kp = 400;
                        kd = 5;
                } else if (jointIdx == 12) {
                        pos = 0.267;
                } else if (jointIdx == 13) {
                        pos = -0.267;
                }

                int index = getJointsIndex(walkjointNames[jointIdx]);
                scalar_t pos_des;
                std::array<MotorState, 24> joint_state = get_joint_state();
                double cur_pos = joint_state[index].pos;
                if (abs(pos - cur_pos) > 0.5)
                        pos_des = 0.98 * cur_pos + 0.02 * pos;
                else
                        pos_des = 0.95 * cur_pos + 0.05 * pos;
                motorcmd[index].pos = pos_des;
                motorcmd[index].kp = kp;
                motorcmd[index].kd = kd;
                motorcmd[index].motor_id = index;
                motorcmd[index].vel = 0;
                motorcmd[index].tau = 0;
                // hw.set_joint(motorcmd);
        }
        for (int j = 0; j < 12; j++) {
                int jointIdx = jointIndicesLeg[j];

                int index = getJointsIndex(walkjointNames[jointIdx]);
                scalar_t action_value = actions_[j] * action_scale[j];
                scalar_t pos_des = action_value + defaultJointAngles_(j);
                double stiffness = joint_stiffness[j]; // 根据关节名称获取刚度
                double damping = joint_damping[j];     // 根据关节名称获取阻尼
                // std::cout << "joint_name:" << partName << "kp:" << stiffness
                // << " kd:" << damping << std::endl;
                motorcmd[index].pos = pos_des;
                motorcmd[index].kp = stiffness;
                motorcmd[index].kd = damping;
                motorcmd[index].motor_id = index;
                motorcmd[index].vel = 0;
                motorcmd[index].tau = 0;
                // hw.set_joint(motorcmd);
                lastActions_(j, 0) = actions_[j];
        }
        set_joint(motorcmd);
        count++;
        return true;
}

void LowController::process() {
        static int keyflag[14];
        if (initfinish == 0)
                return;
        Command cmd;
        auto now = Clock::now();
        long starttimestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();

        const std::shared_ptr<const std::array<MotorState, 24>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                const std::shared_ptr<const joydata> jdata =
                    joy_buffer_.GetData();
                if (jdata) {
                        memcpy(remote_data.button, &(*jdata).button[0],
                               sizeof(remote_data.button));
                        memcpy(remote_data.axes, &(*jdata).axes[0],
                               sizeof(remote_data.axes));
                        cmd.x = remote_data.axes[1];
                        cmd.y = 0;
                        cmd.yaw = remote_data.axes[0];

                        if ((remote_data.button[9] == 1) && (keyflag[9] == 0)) {
                                if (!startcontrol) {
                                        startcontrol = true;
                                        standPercent = 0;
                                        mode_ = WorkMode::LIE;
                                        keyflag[9] = 1;
                                        std::array<MotorState, 24> joint_state =
                                            get_joint_state();
                                        int index = 0;
                                        for (size_t i = 0; i < actuatedDofNum_;
                                             i++) {
                                                index = getJointsIndex(
                                                    jointNames[i]);
                                                currentJointAngles_[i] =
                                                    joint_state[index].pos;
                                        }
                                        printf("start control\n");

                                } else {
                                        startcontrol = false;
                                        mode_ = WorkMode::DEFAULT;
                                        keyflag[9] = 1;
                                        printf("stop control\n");
                                }

                        } else if (remote_data.button[9] == 0)
                                keyflag[9] = 0;
                        if ((remote_data.button[10] == 1) &&
                            (remote_data.button[2] == 1) &&
                            (keyflag[10] == 0)) {
                                if (startcontrol == true) {
                                        if (mode_ != WorkMode::STAND) {
                                                standPercent = 0;
                                                mode_ = WorkMode::STAND;
                                                std::array<MotorState, 24>
                                                    joint_state =
                                                        get_joint_state();
                                                int index = 0;
                                                for (size_t i = 0;
                                                     i < actuatedDofNum_; i++) {
                                                        index = getJointsIndex(
                                                            jointNames[i]);
                                                        currentJointAngles_[i] =
                                                            joint_state[index]
                                                                .pos;
                                                }
                                                printf("STAND2LIE\n");
                                        } else if (mode_ == WorkMode::LIE) {
                                                standPercent = 0;
                                                mode_ = WorkMode::STAND;
                                                printf("LIE2STAND\n");
                                        }
                                }
                        } else if (remote_data.button[10] == 0)
                                keyflag[10] = 0;
                        if ((remote_data.button[5] == 1) &&
                            (remote_data.button[2] == 1) && (keyflag[5] == 0)) {

                                if (mode_ == WorkMode::STAND) {
                                        standPercent = 0;
                                        isChangeMode_ = true;
                                        mode_ = WorkMode::USERMODE;
                                        keyflag[5] = 1;
                                        printf("TO USERWALK MODE\n");
                                }

                        } else if (remote_data.button[5] == 0)
                                keyflag[5] = 0;
                        if ((remote_data.button[11] == 1) &&
                            (keyflag[11] == 0)) {
                                if (mode_ == WorkMode::USERMODE) {
                                        isChangeMode_ = true;
                                        mode_ = WorkMode::STAND;
                                        printf("WALK2STAND\n");

                                } else if (mode_ == WorkMode::DEFAULT) {
                                        standPercent = 0;
                                        printf("deftolie\n");
                                        isChangeMode_ = true;
                                        mode_ = WorkMode::LIE;
                                }
                        }
                }

                switch (mode_) {
                case WorkMode::STAND:
                        handleStandMode();
                        break;
                case WorkMode::LIE:
                        handleLieMode();
                        break;
                case WorkMode::DEFAULT:
                        handleDefautMode();
                        break;
                case WorkMode::USERMODE:
                        setparameter(cmd, &isChangeMode_);
                        handleUserMode();
                        break;
                default:
                        printf("Unexpected mode encountered: %d\n",
                               static_cast<int>(mode_));
                        break;
                }
        }
}

void LowController::computeActions() {
        std::vector<Ort::Value> policyInputValues;
        policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
            memoryInfo, policyObservations_.data(), policyObservations_.size(),
            policyInputShapes_[0].data(), policyInputShapes_[0].size()));
        // run inference
        Ort::RunOptions runOptions;
        std::vector<Ort::Value> outputValues = policySessionPtr->Run(
            runOptions, policyInputNames_.data(), policyInputValues.data(), 1,
            policyOutputNames_.data(), 1);
        if (isfirstCompAct_) {
                for (int i = 0; i < policyObservations_.size(); ++i) {
                        std::cout << policyObservations_[i] << " ";
                        if ((i + 1) % observationSize_ == 0) {
                                std::cout << std::endl;
                        }
                }
                isfirstCompAct_ = false;
        }

        for (int i = 0; i < actionsSize_; i++) {
                actions_[i] =
                    *(outputValues[0].GetTensorMutableData<tensor_element_t>() +
                      i);
        }
}

void LowController::onnxdatainit() {
        Ort::AllocatorWithDefaultOptions allocator;
        // ROS_INFO_STREAM("count: " <<
        // poliycfg->policySessionPtr->GetOutputCount());
        for (int i = 0; i < policySessionPtr->GetInputCount(); i++) {
                auto policyInputnamePtr =
                    policySessionPtr->GetInputNameAllocated(i, allocator);
                policyInputNodeNameAllocatedStrings.push_back(
                    std::move(policyInputnamePtr));
                policyInputNames_.push_back(
                    policyInputNodeNameAllocatedStrings.back().get());
                // inputNames_.push_back(sessionPtr_->GetInputNameAllocated(i,
                // allocator).get());
                policyInputShapes_.push_back(
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }

        for (int i = 0; i < policySessionPtr->GetOutputCount(); i++) {
                auto policyOutputnamePtr =
                    policySessionPtr->GetOutputNameAllocated(i, allocator);
                policyOutputNodeNameAllocatedStrings.push_back(
                    std::move(policyOutputnamePtr));
                policyOutputNames_.push_back(
                    policyOutputNodeNameAllocatedStrings.back().get());
                // outputNames_.push_back(sessionPtr_->GetOutputNameAllocated(i,
                // allocator).get());
                std::cout << policySessionPtr
                                 ->GetOutputNameAllocated(i, allocator)
                                 .get()
                          << std::endl;
                policyOutputShapes_.push_back(
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }
}

bool LowController::loadModel(std::string modelpath) {
        std::string policyFilePath;
        std::string estFilePath;
        // create session
        Ort::SessionOptions sessionOptions;
        bool ret;
        onnxEnvPrt_.reset(
            new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
        sessionOptions.SetInterOpNumThreads(1);
        if (onnxEnvPrt_ == NULL) {
                printf("onnxEnvPrt_  is null\n");
                return false;
        }

        policyFilePath = modelpath + "/policy_walk.onnx";
        printf("Load Onnx model from path : %s\n", policyFilePath.c_str());

        policySessionPtr = std::make_unique<Ort::Session>(
            *onnxEnvPrt_, policyFilePath.c_str(), sessionOptions);
        if (policySessionPtr == NULL) {
                printf("load run model failed\n");
                return false;
        }

        // get input and output info
        policyInputNames_.clear();
        policyOutputNames_.clear();
        policyInputShapes_.clear();
        policyOutputShapes_.clear();
        estSessionPtr = NULL;
        modelname = "walk";
        command_.resize(3);
        isfirstCompAct_ = true;
        isfirstRecObs_ = NULL;
        count = 0;
        model_type = 0;
        onnxdatainit();
        ret = getmodelparam();
        initfinish = 1;

        printf("Load Onnx run model successfully !!!\n");
        return true;
}

bool LowController::getmodelparam() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string conpath = std::string(buf);
        std::string path = modelname;
        // RobotCfg::InitState &initState = robotconfig.initState;
        RobotCfg::ControlCfg &controlCfg = robotconfig.controlCfg;
        RobotCfg::ObsScales &obsScales = robotconfig.obsScales;

        YAML::Node acconfig = YAML::LoadFile(conpath + "/config/e1_ac.yaml");

        int error = 0;

        action_scale =
            acconfig[modelname]["action_scale"].as<std::vector<double>>();
        default_joint_pos =
            acconfig[modelname]["default_joint_pos"].as<std::vector<double>>();
        joint_damping =
            acconfig[modelname]["joint_damping"].as<std::vector<double>>();
        joint_stiffness =
            acconfig[modelname]["joint_stiffness"].as<std::vector<double>>();
        joint_names =
            acconfig[modelname]["joint_names"].as<std::vector<std::string>>();
        standDuration = 1000;
        standPercent = 0;
        // controlCfg.actionScale =
        // acconfig[modelname]["control"]["action_scale"].as<float>();
        controlCfg.decimation =
            acconfig[modelname]["control"]["decimation"].as<int>();
        controlCfg.cycle_time =
            acconfig[modelname]["control"]["cycle_time"].as<float>();

        robotconfig.clipObs = acconfig[modelname]["normalization"]
                                      ["clip_scales"]["clip_observations"]
                                          .as<double>();
        robotconfig.clipActions =
            acconfig[modelname]["normalization"]["clip_scales"]["clip_actions"]
                .as<double>();

        actionsSize_ = acconfig[modelname]["size"]["actions_size"].as<int>();
        observationSize_ =
            acconfig[modelname]["size"]["observations_size"].as<int>();

        stackSize_ = acconfig[modelname]["size"]["stack_size"].as<int>();

        scalez = acconfig[modelname]["axis_mappings"]["scalez"].as<float>();
        scaley = acconfig[modelname]["axis_mappings"]["scaley"].as<float>();
        scalex = acconfig[modelname]["axis_mappings"]["scalex"].as<float>();

        actions_.resize(actionsSize_);

        actuatedDofNum_ = 24;

        policyObservations_.resize(observationSize_ * stackSize_);

        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
        lastActions_.resize(actionsSize_);
        lastActions_.setZero();
        const int inputSize = stackSize_ * observationSize_;
        proprioHistoryBuffer_.resize(inputSize);
        defaultJointAngles_.resize(actionsSize_);
        walkdefaultJointAngles_.resize(actionsSize_);
        for (int i = 0; i < actionsSize_; i++) {
                defaultJointAngles_(i) = default_joint_pos[i];
                // printf("defaultJointAngles[%d]
                // %f\n",i,modelcfg->defaultJointAngles_(i));
        }

        return true;
}

void LowController::computeObservation() {
        std::atomic<scalar_t> comm_x;
        std::atomic<scalar_t> comm_y;
        std::atomic<scalar_t> comm_z;

        vector_t command(3);
        comm_x = command_[0] * scalex;
        comm_y = command_[1] * scaley;
        comm_z = command_[2] * scalez;

        // 绝对值小于0.3的都置0
        if (abs(comm_x) < 0.3)
                comm_x = 0.0;
        if (abs(comm_y) < 0.3)
                comm_y = 0.0;
        if (abs(comm_z) < 0.3)
                comm_z = 0.0;
        if (comm_x < 0)
                comm_z = 0;
        // 如果y的command大于0.3，其余command为0
        if (abs(comm_y) > 0.3) {
                comm_x = 0.0;
                comm_z = 0.0;
        }
        if (abs(comm_x) > 0.3 && abs(comm_z) > 0.3) {
                comm_y = 0.0;
        }

        // x的command限定在-1.0-1.2
        if (comm_x < -1.0)
                comm_x = -1.0;
        // if (command_.x > 1.2) command_.x = 1.2;

        command[0] = comm_x;
        command[1] = comm_y;
        command[2] = comm_z;
        std::vector<int> jointIndices = {
            0, 1, 3, 4, 6, 7, 10, 11, 14, 15, 18, 19,
        };
        vector_t jointPosSel(jointIndices.size());
        vector_t jointVelSel(jointIndices.size());
        for (size_t i = 0; i < jointIndices.size(); i++) {
                int idx = jointIndices[i];
                jointPosSel[i] = propri_.jointPos[idx];
                jointVelSel[i] = propri_.jointVel[idx];
                // defaultAngleSel[i] = defaultJointAngles_[idx];
        }
        vector_t actions(lastActions_);
        vector_t proprioObs(observationSize_);

        proprioObs << command,                   // 3
            propri_.baseAngVel,                  // 3
            propri_.projectedGravity(0),         // 1
            propri_.projectedGravity(1),         // 1
            propri_.projectedGravity(2),         // 1
            (jointPosSel - defaultJointAngles_), // 12
            jointVelSel,                         // 12
            actions;                             // 12

        if (*isfirstRecObs_) {
                for (int i = observationSize_ - actionsSize_;
                     i < observationSize_; i++) {
                        proprioObs(i, 0) = 0.0;
                }

                for (size_t i = 0; i < stackSize_; i++) {
                        proprioHistoryBuffer_.segment(i * observationSize_,
                                                      observationSize_) =
                            proprioObs.cast<tensor_element_t>();
                }
                *isfirstRecObs_ = false;
                std::fill(policyObservations_.begin(),
                          policyObservations_.end(), 0.0f);
        }
        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() -
                                   observationSize_) =
            proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() -
                                       observationSize_);
        proprioHistoryBuffer_.tail(observationSize_) =
            proprioObs.cast<tensor_element_t>();

        //clang-format on

        for (size_t i = 0; i < (observationSize_ * stackSize_); i++) {
                policyObservations_[i] =
                    static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
        }
        // Limit observation range
        scalar_t obsMin = -robotconfig.clipObs;
        scalar_t obsMax = robotconfig.clipObs;
        std::transform(policyObservations_.begin(), policyObservations_.end(),
                       policyObservations_.begin(),
                       [obsMin, obsMax](scalar_t x) {
                               return std::max(obsMin, std::min(obsMax, x));
                       });
}

} // namespace legged

int main() {
        // setenv("CYCLONEDDS_URI","file:///home/oem/test/dds-test/config/dds.xml",1);
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        legged::LowController lowcontroller;
        lowcontroller.init(legged::ControlMode::LOWMODE);
        lowcontroller.loadModel(path + "/policy");
        while (1) {
                usleep(10);
        }
        return 0;
}
