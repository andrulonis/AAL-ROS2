ctmc

module basic
    s: [0..1] init 0;
    [] s=0 -> 1:(s'=1);
    [] s=1 -> 1:(s'=0);
endmodule