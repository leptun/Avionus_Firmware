#pragma once
#include <modules/gps.hpp>
#include <modules/krpc_client.hpp>
#include <modules/servo.hpp>
#include <modules/krpc_client.hpp>

namespace modules {

extern servo::Servo sv;
extern krpc_client::KrpcClient krpc;

}
