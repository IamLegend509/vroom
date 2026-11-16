// Closed-loop fixture: actual RTL PWM -> averaged inverter -> dq motor ODE ->
// phase currents + quantized AS5600 -> RTL. Parameters are TEST ASSUMPTIONS,
// not measured 2805 data. Double-precision RK4, 50us physics integration.
#include "Vfocmini_driver.h"
#include "verilated.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

constexpr double pi=3.14159265358979323846, dt=0.00005;
constexpr uint64_t hz=100000000, pwm_period=5000, frame_period=10000;
using State=std::array<double,4>; // id, iq, mechanical speed, mechanical angle
struct Plant {
    double r=1.0,l=.001,j=.001,ke=.02,kt=.03,b=.001;
    State x{};
    State rhs(const State& y,double a,double bphase,double c,double load) const {
        const double sn=std::sin(7*y[3]),cs=std::cos(7*y[3]);
        const double alpha=(2*a-bphase-c)/3, beta=(bphase-c)/std::sqrt(3.0);
        const double vd=alpha*cs+beta*sn,vq=beta*cs-alpha*sn,we=7*y[2];
        return {(vd-r*y[0]+we*l*y[1])/l,
                (vq-r*y[1]-we*l*y[0]-ke*y[2])/l,
                (kt*y[1]-this->b*y[2]-load)/j,y[2]};
    }
    void step(double a,double bphase,double c,double load) {
        auto shifted=[](const State& y,const State& k,double h) {
            State z{};for(unsigned i=0;i<4;++i)z[i]=y[i]+h*k[i];return z;
        };
        auto k1=rhs(x,a,bphase,c,load);
        auto k2=rhs(shifted(x,k1,dt/2),a,bphase,c,load);
        auto k3=rhs(shifted(x,k2,dt/2),a,bphase,c,load);
        auto k4=rhs(shifted(x,k3,dt),a,bphase,c,load);
        for(unsigned i=0;i<4;++i)x[i]+=dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
    }
    std::array<double,3> currents() const {
        double a=x[0]*std::cos(7*x[3])-x[1]*std::sin(7*x[3]);
        double beta=x[0]*std::sin(7*x[3])+x[1]*std::cos(7*x[3]);
        return {a,-.5*a+std::sqrt(3.0)/2*beta,-.5*a-std::sqrt(3.0)/2*beta};
    }
};
static uint32_t q(double x){return uint32_t(int32_t(std::llround(x*65536)));}
static double unq(uint32_t x){return double(int32_t(x))/65536;}
int main(int argc,char** argv) {
    Verilated::commandArgs(argc,argv);
    if(argc!=3){std::cerr<<"usage: plant FIXTURE TRACE.csv\n";return 2;}
    const bool heavy=std::string(argv[1])=="heavy_reverse";
    Plant motor;
    if(heavy){motor.r=1.5;motor.l=.002;motor.j=.0015;motor.ke=.01;motor.kt=.015;}
    Vfocmini_driver dut;
    std::ofstream trace(argv[2]);
    trace<<"time_s,target_rad_s,omega_rad_s,ia_a,ib_a,ic_a,iq_ref_a,vd_v,vq_v,enable,fault\n";
    dut.clk=0;dut.reset=1;dut.fault_n=1;dut.arm=0;dut.stop=0;dut.target_valid=0;
    dut.target_velocity=0;dut.calibration_valid=1;dut.angle_reverse=heavy;
    dut.electrical_zero=0;dut.phase_trip=q(1);dut.sample_valid=0;dut.sensor_ok=1;
    dut.phase_a=0;dut.phase_b=0;dut.angle_raw=0;
    std::array<double,3> sum_error{},sum_square{};
    std::array<uint64_t,3> counts{};
    double peak_current=0,peak_speed=0;
    uint64_t high_a=0,high_b=0,high_c=0;
    bool started=false;
    try {
        const uint64_t end=205000000;
        for(uint64_t cycle=0;cycle<end+100;++cycle) {
            const double t=double(cycle)/hz;
            double target=t<.7?1.0:t<1.4?-1.0:.5;
            dut.reset=cycle<8;dut.arm=cycle==4000000;
            dut.stop=cycle==end;
            dut.target_valid=(cycle==4000002 || (cycle>4000002 && cycle%5000000==0) ||
                              cycle==70000000 || cycle==140000000);
            dut.target_velocity=q(target);
            dut.sample_valid=cycle%frame_period==0;
            if(dut.sample_valid) {
                auto current=motor.currents();
                dut.phase_a=q(current[0]);dut.phase_b=q(current[1]);
                double theta=heavy?-motor.x[3]:motor.x[3];
                int raw=int(std::llround(theta*4096/(2*pi)))%4096;
                dut.angle_raw=uint32_t((raw+4096)%4096);
            }
            dut.clk=0;dut.eval();dut.clk=1;dut.eval();
            if(dut.fault_latched)throw std::runtime_error("RTL fault "+std::to_string(dut.fault_code)+" at "+std::to_string(t));
            if(cycle>4000002 && cycle<end && dut.enable)started=true;
            if(cycle>=end && dut.enable)throw std::runtime_error("STOP did not cut EN");
            if(std::abs(unq(dut.reference_q))>.50001 || std::abs(unq(dut.voltage_d))>1.50001 || std::abs(unq(dut.voltage_q))>1.50001)
                throw std::runtime_error("RTL output limit");
            high_a+=dut.pwm_a;high_b+=dut.pwm_b;high_c+=dut.pwm_c;
            if(cycle%pwm_period==pwm_period-1) {
                motor.step(9.0*double(high_a)/pwm_period,9.0*double(high_b)/pwm_period,
                           9.0*double(high_c)/pwm_period,t>=1.5?.001:0);
                high_a=high_b=high_c=0;
                auto current=motor.currents();
                for(double a:current)peak_current=std::max(peak_current,std::abs(a));
                peak_speed=std::max(peak_speed,std::abs(motor.x[2]));
                if(!std::isfinite(motor.x[2]) || peak_speed>5 || peak_current>1)
                    throw std::runtime_error("Plant fixture bound exceeded");
                int window=t>=.55&&t<.7?0:t>=1.25&&t<1.4?1:t>=1.9&&t<2.05?2:-1;
                if(window>=0){double e=target-motor.x[2];sum_error[window]+=e;sum_square[window]+=e*e;++counts[window];}
                trace<<std::setprecision(10)<<t<<','<<target<<','<<motor.x[2]<<','<<current[0]<<','<<current[1]<<','<<current[2]<<','
                     <<unq(dut.reference_q)<<','<<unq(dut.voltage_d)<<','<<unq(dut.voltage_q)<<','<<unsigned(dut.enable)<<','<<unsigned(dut.fault_code)<<'\n';
            }
        }
        if(!started)throw std::runtime_error("Motor never enabled");
        bool passed=true;
        std::cout<<"{\"fixture\":\""<<argv[1]<<"\",\"test_assumptions_only\":true,\"r_ohm\":"<<motor.r<<",\"l_h\":"<<motor.l
                 <<",\"j_kgm2\":"<<motor.j<<",\"ke_vs_rad\":"<<motor.ke<<",\"kt_nm_a\":"<<motor.kt<<",\"b_nms_rad\":"<<motor.b
                 <<",\"peak_current_a\":"<<peak_current<<",\"peak_speed_rad_s\":"<<peak_speed<<",\"tail_windows\":[";
        for(unsigned i=0;i<3;++i) {
            if(!counts[i])throw std::runtime_error("Missing tail samples");
            const double mean=sum_error[i]/counts[i],rmse=std::sqrt(sum_square[i]/counts[i]);
            passed=passed&&std::abs(mean)<.15&&rmse<.4;
            if(i)std::cout<<',';
            std::cout<<"{\"samples\":"<<counts[i]<<",\"mean_error_rad_s\":"<<mean<<",\"rmse_rad_s\":"<<rmse<<'}';
        }
        std::cout<<"],\"status\":\""<<(passed?"passed":"failed")<<"\"}\n";
        return passed?0:1;
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
