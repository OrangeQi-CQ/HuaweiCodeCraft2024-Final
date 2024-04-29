#pragma once
#include "cargo.hpp"
#include "config.hpp"
#include "land_controller.hpp"
#include "land_scheduler.hpp"
#include "map.hpp"
#include "ocean_controller.hpp"
#include "ocean_scheduler.hpp"
#include "robot.hpp"
#include "ship.hpp"
#include "utils.hpp"


static void CheckOK();

static void InputFrame();

static void PlanFrame();

static void OutputFrame();

static void BuyRobotShip();

void Init();
void Control();