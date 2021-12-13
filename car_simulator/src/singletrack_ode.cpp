#include "singletrack_ode.h"

#include <boost/math/special_functions/sign.hpp>

singletrack_ode::singletrack_ode(double deltaT, tyreModel tyre_model) : dt(deltaT),
    t(0.0), state(5), Vx(0.0), delta(0.0), alphaf(0.0), alphar(0.0), Fyf(0.0), Fyr(0.0),
    vehicleParams_set(false)
{
    // state = [ r, beta, x, y, psi ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
    state[3] = 0.0;
    state[4] = 0.0;

    // Tyre model
    this->tyre_model = tyre_model;
}

void singletrack_ode::setInitialState(double r0, double beta0, double x0, double y0, double psi0)
{
    // Initial state values
    state[0] = r0;
    state[1] = beta0;
    state[2] = x0;
    state[3] = y0;
    state[4] = psi0;
}

void singletrack_ode::setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz)
{
    // Initialize vehicle parameters
    this->m = m;
    this->a = a;
    this->b = b;
    this->Cf = Cf;
    this->Cr = Cr;
    this->mu = mu;
    this->Iz = Iz;

    vehicleParams_set = true;
}

void singletrack_ode::setReferenceCommands(double velocity, double steer)
{
    Vx    = velocity;
    delta = steer;
}

void singletrack_ode::integrate()
{
    // Check vehicle parameters are set
    if (!vehicleParams_set) {
        throw std::invalid_argument( "Vehicle parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&singletrack_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update time and steering
    t += dt;
}

void singletrack_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Handle zero velocity issue
    if (std::abs(Vx)<=0.01) {
        Vx = 0.01;
    }

    // Actual state
    const double r    = state[0];
    const double beta = state[1];
    const double x    = state[2];
    const double y    = state[3];
    const double psi  = state[4];

    // Slip angles
    alphaf = beta+a*r/Vx-delta;
    alphar = beta-b*r/Vx;

    // Tyre forces
    double Fzf = m*9.81*b/(a+b);
    double Fzr = m*9.81*a/(a+b);
    double zf = std::tan(alphaf);
    double zr = std::tan(alphar);
    double zf_sl = 3*mu*Fzf/Cf;
    double zr_sl = 3*mu*Fzr/Cr;

    switch (tyre_model)
    {
        case LINEAR:
            Fyf = -Cf*alphaf;
            Fyr = -Cr*alphar;
            break;

        case FIALA_WITH_SATURATION:
            if (std::abs(zf)<zf_sl) {
                Fyf = Cf*zf*(-1+std::abs(zf)/zf_sl-std::pow(zf,2.0)/(3*std::pow(zf_sl,2.0)));
            }
            else {
                Fyf = -mu*Fzf*(double)sign(alphaf);
            }
            if (std::abs(zr)<zr_sl) {
                Fyr = Cr*zr*(-1+std::abs(zr)/zr_sl-std::pow(zr,2.0)/(3*std::pow(zr_sl,2.0)));
            }
            else {
                 Fyr = -mu*Fzr*(double)sign(alphar);
            }
            break;

        case FIALA_WITHOUT_SATURATION:
            Fyf = Cf*zf*(-1+std::abs(zf)/zf_sl-std::pow(zf,2.0)/(3*std::pow(zf_sl,2.0)));
            Fyr = Cr*zr*(-1+std::abs(zr)/zr_sl-std::pow(zr,2.0)/(3*std::pow(zr_sl,2.0)));
            break;

        default:
            Fyf = 0.0;
            Fyr = 0.0;

            throw std::invalid_argument( "Uknown tyre model!" );
            break;
    }

    // Vehicle equations
    dstate[0] = (a*Fyf-b*Fyr)/Iz;                       // dr
    dstate[1] = (Fyf+Fyr)/(m*Vx)*std::cos(beta)-r;      // dbeta
    dstate[2] = Vx*std::cos(psi);                       // dx
    dstate[3] = Vx*std::sin(psi);                       // dy
    dstate[4] = r;                                      // dpsi

    // Other variables
    ay = Vx*dstate[1]+r*Vx;
}
