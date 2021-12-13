#include <queue>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulation of a single-track dynamic model, rear-wheel drive, with different tyre models
// Control inputs: longitudinal velocity, steer

class singletrack_ode
{
public:
    typedef enum e_tyreModel {
      LINEAR = 0,
      FIALA_WITH_SATURATION = 1,
      FIALA_WITHOUT_SATURATION = 2
    } tyreModel;

    singletrack_ode(double deltaT, tyreModel tyre_model);

    void setInitialState(double r0, double beta0, double x0, double y0, double psi0);
    void setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz);

    void integrate();
    
    void setReferenceCommands(double velocity, double steer);
    
    void getPose(double &x, double &y, double &psi) { x = state[2]; y = state[3]; psi = state[4]; };
    void getLateralDynamics(double &ay, double &yawrate, double &vy) { ay = this->ay; yawrate = state[0]; vy = state[1]; };
    void getSideslip(double &sideslip) { sideslip = state[1]; }
    void getSlip(double &slip_front, double &slip_rear) { slip_front = alphaf; slip_rear = alphar; }
    void getLateralForce(double &force_front, double &force_rear) { force_front = Fyf; force_rear = Fyr; }
    void getCommands(double &velocity, double &steer) { velocity = Vx; steer = delta; };
    void getTime(double &time) { time = t; };

private:
    // Simulator and integrator variables
    double t, dt;
    double Vx, delta;
    double sideslip, ay, alphaf, alphar, Fyf, Fyr;
    double m, a, b, Cf, Cr, mu, Iz;

    bool vehicleParams_set;
    tyreModel tyre_model;

    state_type state;
    runge_kutta_dopri5 < state_type > stepper;

    // ODE function
    void vehicle_ode(const state_type &state, state_type &dstate, double t);
};
