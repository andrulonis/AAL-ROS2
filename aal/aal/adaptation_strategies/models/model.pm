ctmc

const double crash_rate = speed/(safety+0.01);

module frog
    state : [0..1] init 0;
    [crash] state=0 -> crash_rate : (state'=1);
    [nocrash] state=0 -> 1/crash_rate : (state'=0);
    [] state=1 -> 1 : (state'=0);
endmodule

rewards "crash"
    [crash] true : 1;
endrewards
const double safety = 0.03454793617129326;const double speed = 0.26;