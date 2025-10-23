#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

struct KalmanFilter
{ 
    float angle;    // Current angle estimate
    float bias;     // Gyro bias
            
    /*         Error covariance matrix:           (how strongly we adjust bias
    angle estim. variance | angle/bias covariance  based on measurement)
    ----------------------------------------------
    angle/bias covariance | bias estimate variance    */
    float P[2][2];

    // Gains
    float Q_angle;
    float Q_bias;
    float R_measure;

    void init (float angle, float q_angle, float q_bias, float r_measure)
    {
      this->angle = 0;
      this->bias = 0;
      this->P[0][0] = this->P[0][1] = this->P[1][0] = this->P[1][1] = 0;
      this->Q_angle = q_angle;
      this->Q_bias = q_bias;
      this->R_measure = r_measure;
    }

    float update (float newAngle, float newRate, float dt)
    {
        // Predict
        angle += dt * (newRate - bias);
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Update
        float y = newAngle - angle;
        // High S = final equation will trust measurement less (high rel. noise)
        float S = P[0][0] + R_measure;
        float K[2]; // Angle and bias
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Correct predicted angle by fraction of measurement residual
        // Fraction = predicted angle uncertainty / total uncertainty
        angle += K[0] * y;
        bias += K[1] * y;

        float P00 = P[0][0];
        float P01 = P[0][1];

        P[0][0] -= K[0] * P00;
        P[0][1] -= K[0] * P01;
        P[1][0] -= K[1] * P00;
        P[1][1] -= K[1] * P01;

        return angle;
    }
};

#endif
