//****************************************************************************
// Model: game.qm
// File:  ./game.h
//
// This code has been generated by QM tool (see state-machine.com/qm).
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//****************************************************************************
//${.::game.h} ...............................................................
#ifndef game_h
#define game_h

namespace GAME {

enum GameSignals { // signals used in the game
    TIME_TICK_SIG = QP::Q_USER_SIG, // published from tick ISR
    PLAYER_TRIGGER_SIG, // published by Player (ISR) to trigger the Missile
    PLAYER_QUIT_SIG,    // published by Player (ISR) to quit the game
    GAME_OVER_SIG,      // published by Ship when it finishes exploding

    // insert other published signals here ...
    MAX_PUB_SIG,        // the last published signal

    PLAYER_SHIP_MOVE_SIG, // posted by Player (ISR) to the Ship to move it


    BLINK_TIMEOUT_SIG,  // signal for Tunnel's blink timeout event
    SCREEN_TIMEOUT_SIG, // signal for Tunnel's screen timeout event

    TAKE_OFF_SIG,       // from Tunnel to Ship to grant permission to take off
    HIT_WALL_SIG,       // from Tunnel to Ship when Ship hits the wall
    HIT_MINE_SIG,       // from Mine to Ship or Missile when it hits the mine
    SHIP_IMG_SIG,       // from Ship to the Tunnel to draw and check for hits
    MISSILE_IMG_SIG,    // from Missile the Tunnel to draw and check for hits
    MINE_IMG_SIG,       // sent by Mine to the Tunnel to draw the mine
    MISSILE_FIRE_SIG,   // sent by Ship to the Missile to fire
    DESTROYED_MINE_SIG, // from Missile to Ship when Missile destroyed Mine
    EXPLOSION_SIG,      // from any exploding object to render the explosion
    MINE_PLANT_SIG,     // from Tunnel to the Mine to plant it
    MINE_DISABLED_SIG,  // from Mine to Tunnel when it becomes disabled
    MINE_RECYCLE_SIG,   // sent by Tunnel to Mine to recycle the mine
    SCORE_SIG, // from Ship to Tunnel to adjust game level based on score

    MAX_SIG             // the last signal (keep always last)
};

enum GameBitmapIds {
    PRESS_BUTTON_BMP,
    SHIP_BMP,
    MISSILE_BMP,
    MINE1_BMP,
    MINE2_BMP,
    MINE2_MISSILE_BMP,
    EXPLOSION0_BMP,
    EXPLOSION1_BMP,
    EXPLOSION2_BMP,
    EXPLOSION3_BMP,
    MAX_BMP
};

// obtain instances of the Mines orthogonal components
QP::QHsm *Mine1_getInst(uint8_t id);
QP::QHsm *Mine2_getInst(uint8_t id);

} // namespace GAME

namespace GAME {


#if ((QP_VERSION < 580) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8)))
#error qpcpp version 5.8.0 or higher required
#endif

//${Events::ObjectPosEvt} ....................................................
class ObjectPosEvt : public QP::QEvt {
public:
    uint8_t x;
    uint8_t y;

public:
    ObjectPosEvt(
        QP::QSignal sig,
        uint8_t x_p,
        uint8_t y_p)
      : QEvt(sig),
        x(x_p),
        y(y_p)
    {
    }
};

} // namespace GAME
namespace GAME {

//${Events::ObjectImageEvt} ..................................................
class ObjectImageEvt : public QP::QEvt {
public:
    uint8_t x;
    int8_t y;
    uint8_t bmp;

#if defined __LP64__
    uint32_t pad;
#endif //  defined __LP64__


public:
    ObjectImageEvt(
        QP::QSignal sig,
        uint8_t x_p,
        uint8_t y_p,
        uint8_t bmp_p)
      : QEvt(sig),
        x(x_p),
        y(y_p),
        bmp(bmp_p)
    {
    }
};

} // namespace GAME
namespace GAME {

//${Events::MineEvt} .........................................................
class MineEvt : public QP::QEvt {
public:
    uint8_t id;

public:
    MineEvt(QP::QSignal sig, uint8_t id_p)
      : QEvt(sig),
        id(id_p)    {
    }
};

} // namespace GAME
namespace GAME {

//${Events::ScoreEvt} ........................................................
class ScoreEvt : public QP::QEvt {
public:
    uint16_t score;

public:
    ScoreEvt(QP::QSignal sig, uint16_t score_p)
      : QEvt(sig),
        score(score_p)    {
    }
};

} // namespace GAME

#define GAME_SCREEN_WIDTH          BSP_SCREEN_WIDTH
#define GAME_SCREEN_HEIGHT         BSP_SCREEN_HEIGHT
#define GAME_MINES_MAX             5U
#define GAME_MINES_DIST_MIN        10U
#define GAME_SPEED_X               1U
#define GAME_MISSILE_SPEED_X       2U
#define GAME_SHIP_X                10U
#define GAME_SHIP_Y                (GAME_SCREEN_HEIGHT / 2U)

// opaque pointers to active objects in the application
namespace GAME {

extern QP::QActive * const AO_Tunnel;

} // namespace GAME
namespace GAME {

extern QP::QActive * const AO_Ship;

} // namespace GAME
namespace GAME {

extern QP::QActive * const AO_Missile;

} // namespace GAME

// helper function for all AOs
namespace GAME {

//${AOs::do_bitmaps_overlap} .................................................
bool do_bitmaps_overlap(
    uint8_t bmp_id1,
    uint8_t x1,
    uint8_t y1,
    uint8_t bmp_id2,
    uint8_t x2,
    uint8_t y2);

} // namespace GAME

#endif  // game_h