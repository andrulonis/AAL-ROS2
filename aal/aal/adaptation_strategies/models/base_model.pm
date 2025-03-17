dtmc

module internal_camera
    s_int: [0..3] init int_cam_used ? 1 : 0; // 0 - not used, 1 - ready, 2 - good picture, 3 - bad picture
    [send_int] s_int=1 -> 1-darkness:(s_int'=2) + darkness:(s_int'=3);
    [good_pic] s_int=2 -> 1:(s_int'=1);
    [] s_int=3 -> 1:(s_int'=1);
endmodule

module external_camera
    s_ext: [0..3] init int_cam_used ? 0 : 1; // 0 - not used, 1 - ready, 2 - good picture, 3 - bad picture
    [send_ext] s_ext=1 -> 1:(s_ext'=2);
    [good_pic_ext] s_ext=2 -> 1:(s_ext'=1);
    [] s_ext=3 -> 1:(s_ext'=1);
endmodule

rewards "pictures"
    [good_pic] true : 1;
    [good_pic_ext] true : 0.1;
endrewards

rewards "energy"
    [send_int] true : 60;
    [send_ext] true : 0;
endrewards
