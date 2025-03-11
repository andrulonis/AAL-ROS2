dtmc // Maybe make it continuous instead

const double max_speed = 0.26;

const double saf_alpha1 = 0.8;
const double saf_alpha2 = 0.8;
const double saf_threshold = 0.15;
const double saf_prob_good = 1 - saf_alpha1 * (speed / max_speed);    // will be learned in future
const double saf_prob_bad = saf_alpha2 * (speed / max_speed);           // will be learned in future

const double pow_alpha1 = 0.8;
const double pow_alpha2 = 0.8;
const double pow_threshold = 4.0;
const double pow_prob_good = 1 - pow_alpha1 * (speed / max_speed);    // will be learned in future
const double pow_prob_bad = pow_alpha2 * (speed / max_speed);           // will be learned in future

const double mov_alpha1 = 0.8;
const double mov_alpha2 = 0.7;
const double mov_threshold = 0.4;
const double mov_prob_good = mov_alpha1 * (speed / max_speed);          // will be learned in future
const double mov_prob_bad = 1 - mov_alpha2 * (speed / max_speed);     // will be learned in future

module saf_qr
    s1: [0..1] init safety > saf_threshold ? 1 : 0; // 0 = QR not met, 1 = QR met
    [] s1=0 -> saf_prob_good : (s1'=1) + (1-saf_prob_good) : (s1'=0);
    [] s1=1 -> saf_prob_bad : (s1'=0) + (1-saf_prob_bad) : (s1'=1);
endmodule

module pow_qr
    s2: [0..1] init power < pow_threshold ? 1 : 0; // 0 = QR not met, 1 = QR met
    [] s2=0 -> pow_prob_good: (s2'=1) + (1-pow_prob_good) : (s2'=0);
    [] s2=1 -> pow_prob_bad : (s2'=0) + (1-pow_prob_bad) : (s2'=1);
endmodule

module mov_qr
    s3: [0..1] init move > mov_threshold ? 1 : 0; // 0 = QR not met, 1 = QR met
    [] s3=0 -> mov_prob_good: (s3'=1) + (1-mov_prob_good) : (s3'=0);
    [] s3=1 -> mov_prob_bad : (s3'=0) + (1-mov_prob_bad) : (s3'=1);
endmodule


//rewards "safety"
//    [safer] true : 1;
//    [unsafe] true : -1;
//endrewards
//
//rewards "power"
//
//endrewards
//
//rewards "movement"
//
//endrewards

