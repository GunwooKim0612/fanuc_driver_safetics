// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <optional>

#include "rmi/packets.hpp"

namespace rmi
{
class RMIConnectionInterface
{
public:
  virtual ~RMIConnectionInterface() = default;

  // 2.2.1 Packet Exchange
  virtual ConnectPacket::Response connect(std::optional<double> timeout) = 0;

  // 2.2.2 Controller Disconnect Packet
  virtual DisconnectPacket::Response disconnect(std::optional<double> timeout) = 0;

  // 2.2.3 Timeout Terminate Packet
  virtual std::optional<TimeoutTerminatePacket> checkTimeoutTerminate() = 0;
  
  // 2.2.4 System Fault Packet
  virtual std::optional<SystemFaultPacket> checkSystemFault() = 0;


  // 2.3.1 Packet To Initialize Remote Motion
  virtual InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout) = 0;

  // 2.3.2 Packet To Abort Remote Motion TP Program
  virtual AbortPacket::Response abort(std::optional<double> timeout) = 0;

  // 2.3.3 Packet To Pause Remote Motion Program
  virtual PausePacket::Response pause(std::optional<double> timeout) = 0;

  // 2.3.4 Packet To Continue Remote Motion Program
  virtual ContinuePacket::Response resume(std::optional<double> timeout) = 0;

  // 2.3.5 Packet To Read Controller Error
  virtual ReadErrorPacket::Response readError(std::optional<double> timeout) = 0;

  // 2.3.7 Packet To Get Controller Status
  virtual StatusRequestPacket::Response getStatus(std::optional<double> timeout) = 0;

  // 2.3.12 Packet To Read Digital Input Port
  virtual ReadDigitalInputPortPacket::Response readDigitalInputPort(uint16_t port_number,
                                                                    std::optional<double> timeout) = 0;

  // 2.3.13 Packet To Write Digital Output Port
  virtual WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port_number, bool port_value,
                                                                    std::optional<double> timeout) = 0;

  // 2.3.15 Packet To Read Current Robot Joint Angles
  virtual ReadJointAnglesPacket::Response readJointAngles(const std::optional<uint8_t>& group,
                                                          std::optional<double> timeout) = 0;

  // 2.3.16 Packet To Set Speed Override
  virtual SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout) = 0;

  // 2.3.18 Packet To Read Position Register Data
  virtual ReadPositionRegisterPacket::Response readPositionRegister(int register_number,
                                                                    std::optional<double> timeout) = 0;

  // 2.3.19 Packet To Write Position Register Data
  virtual WritePositionRegisterPacket::Response writePositionRegister(int register_number,
                                                                      const ConfigurationData& configuration,
                                                                      const PositionData& position,
                                                                      std::optional<double> timeout) = 0;

  // 2.3.20 Packet To Reset Robot Controller
  virtual ResetRobotPacket::Response reset(std::optional<double> timeout) = 0;

  // 2.4.5 Packet To Set Payload Instruction
  virtual SetPayloadPacket::Response setPayloadSchedule(uint8_t payload_schedule_number,
                                                        std::optional<double> timeout) = 0;

  // 2.4.6 Packet To Call A Program
  virtual ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout) = 0;
  virtual ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name) = 0;
  
  // 2.4.13 Packet To Add Joint Motion With Joint Representation
  virtual JointMotionJRepPacket::Response sendJointMotion(JointMotionJRepPacket::Request joint_motion_request,
                                                          std::optional<double> timeout) = 0;
  
  // 2.4.19 Unknown Packet Handling
  virtual std::optional<UnknownPacket> checkUnknownPacket() = 0;
  
  // 5.3.3 Communication Packet
  virtual std::optional<CommunicationPacket> checkCommunicationPacket() = 0;

  // New packets not defined in B-84184EN/03
  virtual ConnectROS2Packet::Response connect_ros2(const std::optional<double> timeout) = 0;

  // New packets not defined in B-84184EN/03
  virtual ReadIOPortPacket::Response readIOPort(const std::string& port_type, int port_number,
                                                std::optional<double> timeout) = 0;
  
  // New packets not defined in B-84184EN/03
  virtual WriteIOPortPacket::Response writeIOPort(int port_number, const std::string& port_type,
                                                  std::variant<int, float> port_value,
                                                  std::optional<double> timeout) = 0;

  // New packets not defined in B-84184EN/03
  virtual ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                          std::optional<double> timeout) = 0;

  // New packets not defined in B-84184EN/03
  virtual WriteVariablePacket::Response writeVariablePacket(const std::string& variable_name,
                                                            std::variant<int, float> value,
                                                            std::optional<double> timeout) = 0;
  
  // New packets not defined in B-84184EN/03
  virtual GetExtendedStatusPacket::Response getExtendedStatus(std::optional<double> timeout) = 0;

  // New packets not defined in B-84184EN/03
  virtual ReadNumericRegisterPacket::Response readNumericRegister(int register_number,
                                                                  std::optional<double> timeout) = 0;

  // New packets not defined in B-84184EN/03
  virtual WriteNumericRegisterPacket::Response writeNumericRegister(int register_number, std::variant<int, float> value,
                                                                    std::optional<double> timeout) = 0;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  // 2.3.14 Packet To Read Current Robot Cartesian Position
  virtual GetCartesianPositionPacket::Response getCartesianPosition(const std::optional<uint8_t>& group,
                                                                    std::optional<double> timeout) = 0;

  // 2.3.17 Packet To Get Current User/Tool Frame Number
  virtual GetUFrameToolFramePacket::Response getUFrameUTool(const std::optional<uint8_t>& group,
                                                            std::optional<double> timeout) = 0;

  // 2.3.6 Packet To Set The Current UFrame-UTool Number
  virtual SetUFrameToolFramePacket::Response setUFrameUTool(int uframe, int utool, const std::optional<uint8_t>& group,
                                                            std::optional<double> timeout) = 0;
  
  // 2.3.8 Packet To Read User Frame Data
  virtual ReadUFrameDataPacket::Response readUFrameData(const std::optional<uint8_t>& group,
                                                        std::optional<double> timeout) = 0;

  // 2.3.9 Packet To Set User Frame Data
  virtual WriteUFrameDataPacket::Response writeUFrameData(uint8_t frame_no, const FrameData& frame,
                                                          const std::optional<uint8_t>& group,
                                                          std::optional<double> timeout) = 0;

  // 2.3.10 Packet To Read User Tool Data
  virtual ReadUToolDataPacket::Response readUToolData(int tool_no, const std::optional<uint8_t>& group,
                                                      std::optional<double> timeout) = 0;

  // 2.3.11 Packet To Set User Tool Data
  virtual WriteUToolDataPacket::Response writeUToolData(uint8_t tool_no, const FrameData& frame,
                                                        const std::optional<uint8_t>& group,
                                                        std::optional<double> timeout) = 0;

  // 2.3.21 Packet To Read Current Tool Center Point Speed
  virtual GetTCPSpeedPacket::Response getTCPSpeed(std::optional<double> timeout) = 0;

  // 2.4.1
  virtual WaitForDINPacket::Response waitForDIN(uint16_t port_number, bool on, std::optional<double> timeout) = 0;
  
  // 2.4.2 Packet To Set User Tool Instruction
  virtual SetUFramePacket::Response setUFrame(uint8_t frame_no, std::optional<double> timeout) = 0;

  // 2.4.3  Packet To Set Tool Instruction
  virtual SetToolFramePacket::Response setUTool(uint8_t tool_no, std::optional<double> timeout) = 0;

  // 2.4.4 Packet To Add Wait Time Instruction
  virtual WaitForTimePacket::Response waitTime(float seconds, std::optional<double> timeout) = 0;

  // 2.4.7 Packet To Add Linear Motion Instruction
  virtual LinearMotionPacket::Response linearMotion(LinearMotionPacket::Request req, std::optional<double> timeout) = 0;

  // 2.4.8 Packet To Add Linear Incremental Motion Instruction
  virtual LinearRelativePacket::Response linearRelative(LinearRelativePacket::Request req,
                                                        std::optional<double> timeout) = 0;
  
  // 2.4.9 Packet To Add Joint Motion Instruction
  virtual JointMotionPacket::Response jointMotion(JointMotionPacket::Request req, std::optional<double> timeout) = 0;
  
  // 2.4.10 Packet To Add Joint Incremental Motion Instruction
  virtual JointRelativePacket::Response jointRelative(JointRelativePacket::Request req,
                                                      std::optional<double> timeout) = 0;

  // 2.4.11 Packet To Add Circular Motion Instruction
  virtual CircularMotionPacket::Response circularMotion(CircularMotionPacket::Request req,
                                                        std::optional<double> timeout) = 0;

  // 2.4.12 Packet To Add Circular Incremental Motion Instruction
  virtual CircularRelativePacket::Response circularRelative(CircularRelativePacket::Request req,
                                                            std::optional<double> timeout) = 0;

  // 2.4.14 Packet To Add Joint Incremental Motion With Joint Representation
  virtual JointRelativeJRepPacket::Response jointRelativeJRep(JointRelativeJRepPacket::Request req,
                                                          std::optional<double> timeout) = 0;
  
  // 2.4.15 Packet To Add Linear Motion With Joint Representation
  virtual LinearMotionJRepPacket::Response linearMotionJRep(LinearMotionJRepPacket::Request req,
                                                            std::optional<double> timeout) = 0;

  // 2.4.16 Packet To Add Linear Incremental Motion With Joint Representation
  virtual LinearRelativeJRepPacket::Response linearRelativeJRep(LinearRelativeJRepPacket::Request req,
                                                                std::optional<double> timeout) = 0;

  // 2.4.17 Packet To Add Spline Motion Instruction
  virtual SplineMotionPacket::Response splineMotion(SplineMotionPacket::Request req, std::optional<double> timeout) = 0;
  
  // 2.4.18 Packet To Add Spline Motion With Joint Representation
  virtual SplineMotionJRepPacket::Response splineMotionJRep(SplineMotionJRepPacket::Request req,
                                                            std::optional<double> timeout) = 0;

  // 5.3.1 ASCII String Packet For Single Group Controller
  // virtual CreateASCIIPacket::Response CreateASCIIP(CreateASCIIPacket::Request req, std::optional<double> timeout) = 0;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
};

class RMIConnection final : public RMIConnectionInterface
{
public:
  explicit RMIConnection(const std::string& robot_ip_address, uint16_t rmi_port = 16001);

  ~RMIConnection() override;

  RMIConnection(const RMIConnection&) = delete;

  RMIConnection& operator=(const RMIConnection&) = delete;

  ConnectPacket::Response connect(std::optional<double> timeout) override;

  ConnectROS2Packet::Response connect_ros2(std::optional<double> timeout) override;

  DisconnectPacket::Response disconnect(std::optional<double> timeout) override;

  InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout) override;

  ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout) override;

  ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name) override;

  StatusRequestPacket::Response getStatus(std::optional<double> timeout) override;

  SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout) override;

  AbortPacket::Response abort(std::optional<double> timeout) override;

  PausePacket::Response pause(std::optional<double> timeout) override;

  ContinuePacket::Response resume(std::optional<double> timeout) override;

  ResetRobotPacket::Response reset(std::optional<double> timeout) override;

  ReadErrorPacket::Response readError(std::optional<double> timeout) override;

  WritePositionRegisterPacket::Response writePositionRegister(int register_number,
                                                              const ConfigurationData& configuration,
                                                              const PositionData& position,
                                                              std::optional<double> timeout) override;
  ReadPositionRegisterPacket::Response readPositionRegister(int register_number, std::optional<double> timeout) override;

  ReadNumericRegisterPacket::Response readNumericRegister(int register_number, std::optional<double> timeout) override;

  WriteNumericRegisterPacket::Response writeNumericRegister(int register_number, std::variant<int, float> value,
                                                            std::optional<double> timeout) override;

  // 2.3.12 Packet To Read Digital Input Port
  ReadDigitalInputPortPacket::Response readDigitalInputPort(uint16_t port_number,
                                                            std::optional<double> timeout) override;

  WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port_number, bool port_value,
                                                            std::optional<double> timeout) override;

  ReadIOPortPacket::Response readIOPort(const std::string& port_type, int port_number,
                                        std::optional<double> timeout) override;

  WriteIOPortPacket::Response writeIOPort(int port_number, const std::string& port_type,
                                          std::variant<int, float> port_value, std::optional<double> timeout) override;

  ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                  std::optional<double> timeout) override;

  WriteVariablePacket::Response writeVariablePacket(const std::string& variable_name, std::variant<int, float> value,
                                                    std::optional<double> timeout) override;

  GetExtendedStatusPacket::Response getExtendedStatus(std::optional<double> timeout) override;

  SetPayloadPacket::Response setPayloadSchedule(uint8_t payload_schedule_number, std::optional<double> timeout) override;

  ReadJointAnglesPacket::Response readJointAngles(const std::optional<uint8_t>& group,
                                                  std::optional<double> timeout) override;

  JointMotionJRepPacket::Response sendJointMotion(JointMotionJRepPacket::Request joint_motion_request,
                                                  std::optional<double> timeout) override;

  std::optional<SystemFaultPacket> checkSystemFault() override;

  std::optional<TimeoutTerminatePacket> checkTimeoutTerminate() override;

  std::optional<CommunicationPacket> checkCommunicationPacket() override;

  std::optional<UnknownPacket> checkUnknownPacket() override;

  template <typename T>
  typename T::Response sendRMIPacket(typename T::Request& request_packet, std::optional<double> timeout);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  GetCartesianPositionPacket::Response getCartesianPosition(const std::optional<uint8_t>& group,
                                                            std::optional<double> timeout) override;
  GetUFrameToolFramePacket::Response getUFrameUTool(const std::optional<uint8_t>& group,
                                                    std::optional<double> timeout) override;
  SetUFrameToolFramePacket::Response setUFrameUTool(int uframe, int utool, const std::optional<uint8_t>& group,
                                                    std::optional<double> timeout) override;
  ReadUFrameDataPacket::Response readUFrameData(const std::optional<uint8_t>& group,
                                                std::optional<double> timeout) override;
  WriteUFrameDataPacket::Response writeUFrameData(uint8_t frame_no, const FrameData& frame,
                                                  const std::optional<uint8_t>& group,
                                                  std::optional<double> timeout) override;
  ReadUToolDataPacket::Response readUToolData(int tool_no, const std::optional<uint8_t>& group,
                                              std::optional<double> timeout) override;
  WriteUToolDataPacket::Response writeUToolData(uint8_t tool_no, const FrameData& frame,
                                                const std::optional<uint8_t>& group,
                                                std::optional<double> timeout) override;
  GetTCPSpeedPacket::Response getTCPSpeed(std::optional<double> timeout) override;
  WaitForDINPacket::Response waitForDIN(uint16_t port_number, bool on, std::optional<double> timeout) override;
  SetUFramePacket::Response setUFrame(uint8_t frame_no, std::optional<double> timeout) override;
  SetToolFramePacket::Response setUTool(uint8_t tool_no, std::optional<double> timeout) override;
  WaitForTimePacket::Response waitTime(float seconds, std::optional<double> timeout) override;
  LinearMotionPacket::Response linearMotion(LinearMotionPacket::Request req, std::optional<double> timeout) override;
  LinearRelativePacket::Response linearRelative(LinearRelativePacket::Request req,
                                                std::optional<double> timeout) override;
  JointMotionPacket::Response jointMotion(JointMotionPacket::Request req, std::optional<double> timeout) override;
  JointRelativePacket::Response jointRelative(JointRelativePacket::Request req, std::optional<double> timeout) override;
  CircularMotionPacket::Response circularMotion(CircularMotionPacket::Request req,
                                                std::optional<double> timeout) override;
  CircularRelativePacket::Response circularRelative(CircularRelativePacket::Request req,
                                                    std::optional<double> timeout) override;
  JointRelativeJRepPacket::Response jointRelativeJRep(JointRelativeJRepPacket::Request req,
                                                      std::optional<double> timeout) override;
  LinearMotionJRepPacket::Response linearMotionJRep(LinearMotionJRepPacket::Request req,
                                                    std::optional<double> timeout) override;
  LinearRelativeJRepPacket::Response linearRelativeJRep(LinearRelativeJRepPacket::Request req,
                                                        std::optional<double> timeout) override;
  SplineMotionPacket::Response splineMotion(SplineMotionPacket::Request req, std::optional<double> timeout) override;
  SplineMotionJRepPacket::Response splineMotionJRep(SplineMotionJRepPacket::Request req,
                                                    std::optional<double> timeout) override;
  // CreateASCIIPacket::Response CreateASCIIP(CreateASCIIPacket::Request req, std::optional<double> timeout) override;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
  struct PConnectionImpl;

  int32_t getSequenceNumber();

  template <typename T>
  T getResponsePacket(std::optional<double> timeout_optional, const std::string& error_message_prefix,
                      std::optional<int> expected_sequence_id);

  template <typename T>
  std::optional<T> checkPushPacket();

  void drainConnectionBuffer();

  const std::string robot_ip_address_;
  const uint16_t rmi_port_;

  int32_t sequence_number_;
  std::list<std::string> json_responses_;
  mutable std::mutex mutex_;
  mutable std::mutex motion_mutex_;

  const std::unique_ptr<PConnectionImpl> connection_impl_;
};

}  // namespace rmi
// TODO: Add doc comments.
