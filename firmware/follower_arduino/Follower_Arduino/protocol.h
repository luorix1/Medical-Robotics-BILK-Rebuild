// shared/protocol.h
#ifndef BILK_PROTOCOL_H
#define BILK_PROTOCOL_H
#include <stdint.h>
#define BILK_P0 'B'
#define BILK_P1 'I'
#define BILK_P2 'L'
#define BILK_P3 'K'
#define BILK_VERSION 0x01
enum BILK_Msg : uint8_t {
  BILK_LeaderState   = 0x01,
  BILK_LeaderHb      = 0x02,
  BILK_CmdSetMode    = 0x10,
  BILK_CmdTrajPoint  = 0x11
};
#pragma pack(push,1)
typedef struct {
  uint32_t t_us;
  float    q[4];
  float    qd[4];
  uint8_t  buttons;
  uint8_t  reserved[3];
} bilk_leader_state_t;
typedef struct {
  uint8_t  mode; // 0=IDLE,1=FOLLOW,2=HOLD,3=SHUTDOWN
  uint8_t  r[3];
} bilk_cmd_set_mode_t;
typedef struct {
  uint32_t t_us;
  float    q_des[4];
  float    qd_des[4];
} bilk_cmd_traj_point_t;
#pragma pack(pop)
#endif
