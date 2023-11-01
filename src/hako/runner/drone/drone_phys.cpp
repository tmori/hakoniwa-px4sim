#include "drone_phys.hpp"
#include <memory.h>
#include <math.h>

void drone_init(double delta_t, const DronePhysParamType&param, const DronePhysStateType& initial_value, DronePhysType& phys)
{
    memset(&phys, 0, sizeof(phys));
    phys.initial_value = initial_value;
    phys.param = param;
    phys.delta_t = delta_t;
    phys.current = initial_value;
    return;
}

static void drone_run_x(double u, DronePhysType& phys);
static void drone_run_y(double u, DronePhysType& phys);
static void drone_run_z(double u, DronePhysType& phys);

static void drone_run_rx(const DronePropellerRotationRateType& propeller, DronePhysType& phys);
static void drone_run_ry(const DronePropellerRotationRateType& propeller, DronePhysType& phys);
static void drone_run_rz(const DronePropellerRotationRateType& propeller, DronePhysType& phys);

void drone_run(const DronePropellerRotationRateType& propeller, DronePhysType& phys)
{
    double u = \
           phys.param.p * ( propeller.w[0] * propeller.w[0]) \
         + phys.param.p * ( propeller.w[1] * propeller.w[1]) \
         + phys.param.p * ( propeller.w[2] * propeller.w[2]) \
         + phys.param.p * ( propeller.w[3] * propeller.w[3]) \
        ;
    drone_run_x(u, phys);
    drone_run_y(u, phys);
    drone_run_z(u, phys);

    drone_run_rx(propeller, phys);
    drone_run_ry(propeller, phys);
    drone_run_rz(propeller, phys);

    phys.current = phys.next;
    phys.current_time += phys.delta_t;
    return;
}

static void drone_run_x(double u, DronePhysType& phys)
{
    phys.next.vec.x = ( phys.delta_t /  phys.param.m ) * u 
                * 
                  ( 
                      cos(phys.current.rot.x)
                    * sin(phys.current.rot.y)
                    * cos(phys.current.rot.z)
                    +
                      sin(phys.current.rot.x)
                    * sin(phys.current.rot.z) 
                  ) 
                  + phys.current.vec.x;

    phys.next.pos.x = (phys.current.vec.x * phys.delta_t) + phys.current.pos.x;
    return;
}

static void drone_run_y(double u, DronePhysType& phys)
{
    phys.next.vec.y = ( phys.delta_t /  phys.param.m ) * u 
                * 
                  ( 
                      cos(phys.current.rot.x) 
                    * sin(phys.current.rot.y) 
                    * cos(phys.current.rot.z) 
                    - 
                      sin(phys.current.rot.x) 
                    * sin(phys.current.rot.z) 
                  ) 
                  + phys.current.vec.y;

    phys.next.pos.y = (phys.current.vec.y * phys.delta_t) + phys.current.pos.y;
    return;
}
static void drone_run_z(double u, DronePhysType& phys)
{
    phys.next.vec.z = ( phys.delta_t /  phys.param.m ) * u 
                * 
                  ( 
                      cos(phys.current.rot.y) 
                    * cos(phys.current.rot.x) 
                  ) 
                  - (phys.param.gravity * phys.delta_t )
                  + phys.current.vec.z;

    phys.next.pos.z = (phys.current.vec.z * phys.delta_t) + phys.current.pos.z;
    /*
     * 境界条件：地面から下には落ちない
     */
    if (phys.next.pos.z < 0) {
      phys.next.pos.z = 0;
    }
    return;
}

static void drone_run_rx(const DronePropellerRotationRateType& propeller, DronePhysType& phys)
{
    double torque_phi = - phys.param.l * phys.param.p * propeller.w[1] * propeller.w[1]
                        + phys.param.l * phys.param.p * propeller.w[3] * propeller.w[3];

    phys.next.rot_vec.x = torque_phi * phys.delta_t + phys.current.rot_vec.x;
    phys.next.rot.x     = (phys.current.rot_vec.x * phys.delta_t) + phys.current.rot.x;
}
static void drone_run_ry(const DronePropellerRotationRateType& propeller, DronePhysType& phys)
{
    double torque_theta = - phys.param.l * phys.param.p * propeller.w[0] * propeller.w[0]
                        + phys.param.l * phys.param.p * propeller.w[2] * propeller.w[2];

    phys.next.rot_vec.y = torque_theta * phys.delta_t + phys.current.rot_vec.y;
    phys.next.rot.y     = (phys.current.rot_vec.y * phys.delta_t) + phys.current.rot.y;
}
static void drone_run_rz(const DronePropellerRotationRateType& propeller, DronePhysType& phys)
{
    double torque_psi   = phys.param.k * propeller.w[0] * propeller.w[0]
                        - phys.param.k * propeller.w[1] * propeller.w[1]
                        + phys.param.k * propeller.w[2] * propeller.w[2]
                        - phys.param.k * propeller.w[3] * propeller.w[3];

    phys.next.rot_vec.z = torque_psi * phys.delta_t + phys.current.rot_vec.z;
    phys.next.rot.z     = (phys.current.rot_vec.z * phys.delta_t) + phys.current.rot.z;
}

