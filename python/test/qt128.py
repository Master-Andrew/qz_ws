import socket
import struct


# Format	          C Type	              Python	    字节数
#          x	    pad byte	    no value	         1
#          c	    char	bytes of length 1	         1
#          b	    signed char	    integer	         1
#          B	   unsigned char	    integer	         1
#          ？	   _Bool	              bool	         1
#          h     short            integer	         2
#          H	   unsigned short	    integer	         2
#          i	   int	              integer	         4
#          I	   unsigned int	    integer	         4
#          l	   long	    integer	         4
#          L	   unsigned long	    integer	         4
#         q	   long long	    integer	         8
#         Q	   unsigned long long	    integer	         8
#         f	   float	    float	         4
#         d	   double	    float	         8
#         s	   char[]	    bytes	         1
#         p	   char[]	    bytes	         1
#        P	   void *	    integer


class LidarConfig:
  def __init__(self, address, mask, gateway, vlan, destination_ip, destination_port, spin_rate, return_mode,
               sync_angle):
    self.address = address
    self.mask = mask
    self.gateway = gateway
    self.vlan = vlan
    self.destination_ip = destination_ip
    self.destination_port = destination_port
    self.spin_rate = spin_rate
    self.return_mode = return_mode
    self.sync_angle = sync_angle


def qt128_setting(config):
  head = "BBBBI"

  control_ip_field = "4B4B4BBH"
  control_ip_setting = struct.pack(head + control_ip_field, 0x47, 0x74, 0x21, 0, 15, config.address, config.mask,
                                   config.gateway, 0, 1)
  # PTC_COMMAND_SET_CONTROL_PORT0x21To set the LiDAR's IPv4, mask, gateway, and VLAN settings
  # IP address of the deviceThe i-th (i = 1~4) byte represents the i-th sectionDefault:192, 168, 1, and 201
  # Subnet mask of the deviceThe i-th (i= 1~4) byte represents the i-th sectionDefault: 255, 255, 255, and 0
  # Gateway of the deviceThe i-th (i = 1~4) byte represents the i-th sectionDefault: 192, 168, 1and 1
  # VLAN Status0-OFF           1-ONNOTENot supported on QT128C2X (always 0)
  # VLAN IDRange: [1, 4094]NOTENot supported on QT128C2X (always 1)

  destination_ip_field = "4BHH"
  destination_setting = struct.pack(head + destination_ip_field, 0x47, 0x74, 0x20, 0, 8, config.destination_ip,
                                    config.destination_port,
                                    10110)
  # PTC_COMMAND_SET_DESTINATION_IP0x20To set the LiDAR's Destination IP and Port
  # Destination IP address of Point Cloud Data PacketsThe i-th (i = 1~4) byte represents the i-th sectionDefault: 255, 255, 255, and 255
  # LiDAR Destination PortDefault: 2368
  # GPS Destination PortDefault: 10110NOTENot configurable on QT128C2X

  spin_rate_field = "H"
  spin_rate_setting = struct.pack(head + spin_rate_field, 0x47, 0x74, 0x17, 0, 2, config.spin_rate)
  # PTC_COMMAND_SET_SPIN_RATE0x17To set the LiDAR's spin rate
  # 600-600 rpm   1200 -1200 rpm

  return_mode_field = "B"
  return_mode_setting = struct.pack(head + return_mode_field, 0x47, 0x74, 0x1e, 0, 1, config.return_mode)
  # PTC_COMMAND_SET_RETURN_MODE0x1eTo set the return mode: last/strongest/dual return
  # 0 -Last return3-First return4-Last and First Return

  sync_angle_field = "BH"
  sync_angle_field = struct.pack(head + sync_angle_field, 0x47, 0x74, 0x20, 0, 3, 1, config.sync_angle)
  # PTC_COMMAND_SET_SYNC_ANGLE0x18To set the sync angle between the LiDAR's 0°position and the PPS signal
  # Sync angle enable flag0 -OFF1 -ON
  # Angle valueRange:[0,359]

  trigger_method_field = "B"
  trigger_method_setting = struct.pack(head + trigger_method_field, 0x47, 0x74, 0x1b, 0, 1, 0)
  # PTC_COMMAND_SET_TRIGGER_METHOD0x1bTo select the method for triggering laser firings: angle/time based
  # 0 -Angle based1 -Time based

  ptp_field = "BBBBBB"
  ptp_setting = struct.pack(head + ptp_field, 0x47, 0x74, 0x24, 0, 6, 0, 1, 0, 0, 1, 1, 0)
  # PTC_COMMAND_SET_PTP_CONFIG0x24To configure PTP settings
  # EEE timing and synchronization standard0 (fixed), representing IEEE 1588v2
  # Domain attribute of the local clockRange: [0, 127]
  # Network transport type of 1588v20 -UDP/IP            1 -L2
  # Time interval between Announce messages, in units of log seconds (default: 1)Range: [-2, 3]
  # Time interval between Sync messages, in units of log seconds (default: 1)Range: [-7, 3]
  # Minimum permitted mean time between Delay_Req messages, in units of log seconds (default: 0)Range: [-7, 3

  standby_mode_field = "B"
  standby_mode_setting = struct.pack(head + standby_mode_field, 0x47, 0x74, 0x1b, 0, 1, 0)
  # PTC_COMMAND_SET_STANDBY_MODE0x1cTo enter/exit standby mode
  # 0 -In operation1 -Standby

  lidar_settings = [control_ip_setting, destination_setting, spin_rate_setting, return_mode_setting, sync_angle_field,
                    trigger_method_setting, ptp_setting, standby_mode_setting]


