use crate::mat_utils::*;

#[derive(Debug)]
/// Extended Kalman Filter for attitude estimation from MARG data
pub struct MargEkf {
    /// Internal state: rotation quaternion and gyroscope biases
    pub state: [[f32; 7]; 1],
    /// Predicted gravity and magnetic vectors
    pub predicted_accel_mag: [[f32; 6]; 1],
    /// A measure of the estimated accuracy of the state estimate
    predicted_estimate_covariance: [[f32; 7]; 7],
    /// Joint variability of process noise
    process_noise_covariance: [[f32; 7]; 7],
    /// Joint variability of observation noise
    observation_noise_covariance: [[f32; 6]; 6],
    /// Optimal Kalman gain
    gain: [[f32; 7]; 6],
    /// Observation model
    observation_model: [[f32; 6]; 7],
    /// Next state
    next_state: [[f32; 7]; 1],
    /// Previous state
    prev_state: [[f32; 7]; 1],
    /// Prediction for next timestamp
    next_estimate_covariance: [[f32; 7]; 7],
    /// Reference vector for acceleration
    acc_ref: [[f32; 3]; 1],
    /// Reference vector for magnetic field
    mag_ref: [[f32; 3]; 1],
}

impl MargEkf {
    pub fn new() -> Self {

        let state = [[1., 0., 0., 0., 0., 0., 0.]];
        MargEkf {
            state: state,
            predicted_accel_mag: [[0., 0., 0., 0., 0., 0.]],
            predicted_estimate_covariance: eye(0.01),
            process_noise_covariance: eye(0.001),
            observation_noise_covariance: eye(0.1),
            gain: Default::default(),
            observation_model: Default::default(),
            next_state: state,
            prev_state: state,
            next_estimate_covariance: eye(0.),
            acc_ref: [[0., 0., -1.]],
            mag_ref: [[0., -1., 0.]],
        }
    }

    pub fn predict(&mut self, w_x: f32, w_y: f32, w_z: f32, d_t: f32) {
        /*
        let q = self.state.slice(s![0..4]);
        let rotation = array![
            [-q[1], -q[2], -q[3]],
            [q[0], -q[3], q[2]],
            [q[3], q[0], -q[1]],
            [-q[2], q[1], q[0]]
        ];
        let advance = d_t / 2.0 * rotation;
        */
        let q = &self.state[0];
        let dt2 = d_t / 2.;

        // NB: row major
        let advance = [
            [-q[1]*dt2, -q[2]*dt2, -q[3]*dt2],
            [ q[0]*dt2, -q[3]*dt2,  q[2]*dt2],
            [ q[3]*dt2,  q[0]*dt2, -q[1]*dt2],
            [-q[2]*dt2,  q[1]*dt2,  q[0]*dt2]
        ];
        /*
        let mut transition_model: Array<f32, Ix2> = Array::eye(7);
        transition_model
            .slice_mut(s![0..4, 4..7])
            .assign(&(-advance.clone()));
        */
        let mut transition_model = eye::<7>(1.0);
        for i in 0..4 {
            for j in 0..3 {
                transition_model[j+4][i] = -advance[i][j]; // transpose!
            }
        }

        /*
        let mut control_model: Array<f32, Ix2> = Array::zeros((7, 3));
        control_model.slice_mut(s![0..4, 0..3]).assign(&advance);
        */
        let mut control_model = [[0.; 7]; 3];
        for i in 0..4 {
            for j in 0..3 {
                control_model[j][i] = advance[i][j]; // transpose!
            }
        }

        /*
        self.next_state = transition_model.dot(&self.state)
            + control_model.dot(&array![w_x, w_y, w_z].reversed_axes());
        */
        mset(&mut self.next_state, 0.);
        mmulm_inc(&transition_model, &self.state, &mut self.next_state);
        mmulm_inc(&control_model, &[[w_x, w_y, w_z]], &mut self.next_state);

        /*
        self.next_estimate_covariance = (transition_model.dot(&self.predicted_estimate_covariance))
            .dot(&transition_model.reversed_axes())
            + &self.process_noise_covariance;
         */
        self.next_estimate_covariance = self.process_noise_covariance;
        let mut tmp = [[0.; 7]; 7];
        mmulm_inc(&transition_model, &self.predicted_estimate_covariance, &mut tmp);
        mmult_inc(&tmp, &transition_model, &mut self.next_estimate_covariance);

        /*
        let quat_mag = self.next_state.slice(s![0..4]).map(|q| q * q).sum().sqrt();
        let normalized_quat = self.next_state.slice(s![0..4]).map(|q| q / quat_mag);
        self.next_state.slice_mut(s![0..4]).assign(&normalized_quat);
         */
        let quat_mag = &self.next_state[0][0..4].iter().map(|q| q*q).sum::<f32>().sqrt();
        for i in 0..4 {
            self.next_state[0][i] /= quat_mag;
        }

        /*
        self.prev_state.assign(&self.state);
         */
        self.prev_state = self.state;

        /*
        let mut observation_model: Array<f32, Ix2> = Array::zeros((6, 7));
        observation_model
            .slice_mut(s![0..3, 0..4])
            .assign(&jacobian(
                &self.prev_state.slice(s![0..4]).into(),
                &self.acc_ref,
            ));
        observation_model
            .slice_mut(s![3..6, 0..4])
            .assign(&jacobian(
                &self.prev_state.slice(s![0..4]).into(),
                &self.mag_ref,
            ));
        self.observation_model = observation_model;
         */
        jacobian(&self.prev_state[0], &self.acc_ref[0], &mut self.observation_model, 0);
        jacobian(&self.prev_state[0], &self.mag_ref[0], &mut self.observation_model, 3);

        /*
        let mut predicted_accel_mag: Array<f32, Ix1> = Array::zeros(6);
        predicted_accel_mag.slice_mut(s![0..3]).assign(
            &rotation_matrix(&self.next_state)
                .reversed_axes()
                .dot(&self.acc_ref),
        );
        predicted_accel_mag.slice_mut(s![3..6]).assign(
            &rotation_matrix(&self.next_state)
                .reversed_axes()
                .dot(&self.mag_ref),
        );
        self.predicted_accel_mag = predicted_accel_mag;
         */
        mset(&mut self.predicted_accel_mag, 0.);

        let mut rot = [[0.; 3]; 3];
        rotation_matrix(&self.next_state[0], &mut rot);

        let mut tmp = [[0.; 3]; 1];
        mmulm_inc(&rot, &self.acc_ref, &mut tmp);
        self.predicted_accel_mag[0][0..3].copy_from_slice(&tmp[0]);
        let mut tmp = [[0.; 3]; 1];
        mmulm_inc(&rot, &self.mag_ref, &mut tmp);
        self.predicted_accel_mag[0][3..6].copy_from_slice(&tmp[0]);
    }

    fn mag_from_raw(&self, raw: [[f32; 3]; 1]) -> [[f32; 3]; 1] {
        // // TODO: those two constants should be part of something else than this crate
        // let a_inv = flip([
        //     [2e-03, -1e-04, -1e-06],
        //     [-1e-04, 1e-03, 1e-05],
        //     [-1e-06, 1e-05, 1e-03]
        // ]);
        // let b = [[80., 37., 105.]];

        // /*
        // let gauss_raw = a_inv.dot(&(raw.reversed_axes() - b));
        // */
        // let mut gauss_raw = [[0.; 3]; 1];
        // mmulm_inc(&a_inv, &msubm(&raw, &b), &mut gauss_raw);

        let gauss_raw = raw;

        /*
        let mut gauss_n = rotation_matrix(&self.state).dot(&gauss_raw);
        gauss_n[2] = 0.;
        let norm = (gauss_n[0].powi(2) + gauss_n[1].powi(2)).sqrt();
        gauss_n = gauss_n / norm;
         */
        let mut rot = [[0.; 3]; 3];
        rotation_matrix(&self.state[0], &mut rot);
        let mut gauss_n = [[0.; 3]; 1];
        mmulm_inc(&rot, &gauss_raw, &mut gauss_n);

        gauss_n[0][2] = 0.;
        let norm = gauss_n[0].iter().map(|v| v * v).sum::<f32>().sqrt();
        gauss_n[0][0] = gauss_n[0][0] / norm;
        gauss_n[0][1] = gauss_n[0][1] / norm;

        /*
        rotation_matrix(&self.state).reversed_axes().dot(&gauss_n)
         */
        let mut out = [[0.; 3]; 1];
        tmulm_inc(&rot, &gauss_n, &mut out);
        out
    }

    pub fn update(&mut self, a: [f32; 3], m: [f32; 3]) {
        /*
        self.gain = self
            .next_estimate_covariance
            .dot(&self.observation_model.view().reversed_axes())
            .dot(&invert(
                self.observation_model
                    .dot(&self.next_estimate_covariance)
                    .dot(&self.observation_model.view().reversed_axes())
                    + &self.observation_noise_covariance,
            ));
         */

        mset(&mut self.gain, 0.);

        let mut tmp = self.observation_noise_covariance;
        mmult_inc(&mmulm(&self.observation_model, &self.next_estimate_covariance),
                  &self.observation_model, &mut tmp);
        mmulm_inc(&mmult(&self.next_estimate_covariance, &self.observation_model),
                  &invert(tmp), &mut self.gain);

        /*
        let magnetic = self.mag_from_raw(array![m[0], m[1], m[2]]);
         */
        let magnetic = self.mag_from_raw([m]);
        /*
        let mut accel = (array![a[0], a[1], a[2]]).reversed_axes();
        let norm = (accel[0].powi(2) + accel[1].powi(2) + accel[2].powi(2)).sqrt();
        accel /= norm;
        let measurement = concatenate![Axis(0), accel, magnetic];
         */
        let norm = a.iter().map(|v| v * v).sum::<f32>().sqrt();
        let measurement = [[a[0] / norm, a[1] / norm, a[2] / norm, magnetic[0][0], magnetic[0][1], magnetic[0][2]]];
        /*
        self.state.assign(&self.next_state);
        self.state += &self.gain.dot(&(measurement - &self.predicted_accel_mag));
         */
        self.state = self.next_state;
        mmulm_inc(&self.gain, &msubm(&measurement, &self.predicted_accel_mag), &mut self.state);

        /*
        let quat_mag = self.state.slice(s![0..4]).map(|q| q * q).sum().sqrt();
        let normalized_quat = self.state.slice(s![0..4]).map(|q| q / quat_mag);
        self.state.slice_mut(s![0..4]).assign(&normalized_quat);
         */
        let quat_mag = &self.state[0][0..4].iter().map(|q| q*q).sum::<f32>().sqrt();
        for i in 0..4 {
            self.state[0][i] /= quat_mag;
        }

        /*
        self.predicted_estimate_covariance = (Array::eye(7)
            - self.gain.dot(&self.observation_model))
        .dot(&self.next_estimate_covariance)
         */

        self.predicted_estimate_covariance = mmulm(
            &msubm(&eye(1.), &mmulm(&self.gain, &self.observation_model)),
            &self.next_estimate_covariance);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

fn mcmpm<const I: usize, const J: usize>(l: &[[f32; I]; J], r: &[[f32; I]; J], eps: f32) -> bool {
    let mut eq = true;
    for i in 0..I {
        for j in 0..J {
            eq = eq && (l[j][i] - r[j][i]).abs() < eps;
        }
    }
    eq
}

    struct Pair<const I: usize, const J: usize> {
        eps: f32,
        l: [[f32; I]; J],
        r: [[f32; I]; J],
    }

    impl<const I: usize, const J: usize> std::fmt::Debug for Pair<I, J> {
        fn fmt(&self, fmt: &mut std::fmt::Formatter) -> std::fmt::Result {
            writeln!(fmt, "")?;
            for j in 0..J {
                write!(fmt, "[ ")?;
                for i in 0..I {
                    let mark = if (self.l[j][i] - self.r[j][i]).abs() >= self.eps { "\x1b[1m" } else { "" };
                    write!(fmt, "{}{}{:12}\x1b[0m", if i > 0 { ", " } else { "" }, mark, self.l[j][i])?;
                }
                write!(fmt, "]   [")?;
                for i in 0..I {
                    let mark = if (self.l[j][i] - self.r[j][i]).abs() >= self.eps { "\x1b[1m" } else { "" };
                    write!(fmt, "{}{}{:12}\x1b[0m", if i > 0 { ", " } else { "" }, mark, self.r[j][i])?;
                }
                writeln!(fmt, "]")?;
            }
            Ok(())
        }
    }

    macro_rules! assert_eq_eps {
        ($eps:expr, $l:expr, $r:expr) => (
            if !mcmpm(&$l, &$r, $eps) {
                panic!("{:?}", Pair { eps: $eps, l: $l, r: $r });
            }
        );
    }

    #[test]
    fn test_ekf_initial_predict() {
        let mut ekf = MargEkf::new();
        ekf.predict(-0.01, 0.02, 0.03, 0.1);
        assert_eq!(
            ekf.next_state,
            [[
                0.99999833,
                -0.0004999992,
                0.0009999984,
                0.0014999975,
                0.0,
                0.0,
                0.0
            ]]
        )
    }

    #[test]
    fn test_ekf_initial_update() {
        let mut ekf = MargEkf::new();
        ekf.predict(-0.01, 0.02, 0.03, 0.1);
        ekf.update(
            [-0.038308594, -0.22745728, 10.362474],
            [342.98535, 165.42484, 331.73993],
        );
        assert_eq_eps!(
            1.0e-8,
            ekf.gain,
            flip([
                [0., 0., -0.117021285, 0., -0.11702128, 0.],
                [0., -0.11716259, 0., 0., 0., 0.117162585],
                [0.15301873, 0., 0., 0., 0., 0.],
                [0., 0., 0., -0.15301873, 0., 0.],
                [0., 0.005313497, 0., 0., 0., -0.005313496],
                [-0.006939625, 0., 0., 0., 0., 0.],
                [0., 0., 0., 0.006939625, 0., 0.]
            ])
        );
        assert_eq_eps!(
            1.0e-7,
            ekf.state,
            [[
                0.97281927,
                0.0035848243,
                0.00019932055,
                -0.23153809,
                -0.00012722983,
                0.000039538067,
                0.006821011
            ]]
        );
        assert_eq_eps!(
            1.0e-8,
            ekf.predicted_estimate_covariance,
            flip([
                [0.005851064, 0., 0., 0., 0., 0., 0.],
                [0., 0.0058581303, 0., 0., -0.00026567484, 0., 0.],
                [0., 0., 0.0076509374, 0., 0., -0.0003469813, 0.],
                [0., 0., 0., 0.0076509374, 0., 0., -0.0003469813],
                [0., -0.0002656748, 0., 0., 0.010989373, 0., 0.],
                [0., 0., -0.00034698128, 0., 0., 0.010993061, 0.],
                [0., 0., 0., -0.00034698128, 0., 0., 0.010993061]
            ])
        );
    }

    #[test]
    fn test_ekf_initial_state() {
        let ekf = MargEkf::new();
        println!("{:?}", ekf);
        assert_eq!(ekf.state, [[1., 0., 0., 0., 0., 0., 0.]]);
    }
}

fn rotation_matrix<const I: usize>(quat: &[f32; I], out: &mut [[f32; 3]; 3]) {
    out[0][0] = quat[0]*quat[0] + quat[1]*quat[1] - quat[2]*quat[2] - quat[3]*quat[3];
    out[0][1] = 2.0 * (quat[1] * quat[2] - quat[0] * quat[3]);
    out[0][2] = 2.0 * (quat[1] * quat[3] + quat[0] * quat[2]);
    out[1][0] = 2.0 * (quat[1] * quat[2] + quat[0] * quat[3]);
    out[1][1] = quat[0]*quat[0] - quat[1]*quat[1] + quat[2]*quat[2] - quat[3]*quat[3];
    out[1][2] = 2.0 * (quat[2] * quat[3] - quat[0] * quat[1]);
    out[2][0] = 2.0 * (quat[1] * quat[3] - quat[0] * quat[2]);
    out[2][1] = 2.0 * (quat[2] * quat[3] + quat[0] * quat[1]);
    out[2][2] = quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2] + quat[3]*quat[3];
}

//x: [[f32; rows]; cols]
//x[col][row]

fn jacobian<const I: usize, const J: usize, const K: usize>(quat: &[f32; I], reference: &[f32; 3], out: &mut [[f32; J]; K], row: usize) {
    out[0][row+0] = 2. * (quat[0] * reference[0] + quat[3] * reference[1] - quat[2] * reference[2]);
    out[1][row+0] = 2. * (quat[1] * reference[0] + quat[2] * reference[1] + quat[3] * reference[2]);
    out[2][row+0] = 2. * (-quat[2] * reference[0] + quat[1] * reference[1] - quat[0] * reference[2]);
    out[3][row+0] = 2. * (-quat[3] * reference[0] + quat[0] * reference[1] + quat[1] * reference[2]);
    out[0][row+1] = 2. * (-quat[3] * reference[0] + quat[0] * reference[1] + quat[1] * reference[2]);
    out[1][row+1] = 2. * (quat[2] * reference[0] - quat[1] * reference[1] + quat[0] * reference[2]);
    out[2][row+1] = 2. * (quat[1] * reference[0] + quat[2] * reference[1] + quat[3] * reference[2]);
    out[3][row+1] = 2. * (-quat[0] * reference[0] - quat[3] * reference[1] + quat[2] * reference[2]);
    out[0][row+2] = 2. * (quat[2] * reference[0] - quat[1] * reference[1] + quat[0] * reference[2]);
    out[1][row+2] = 2. * (quat[3] * reference[0] - quat[0] * reference[1] - quat[1] * reference[2]);
    out[2][row+2] = 2. * (quat[0] * reference[0] + quat[3] * reference[1] - quat[2] * reference[2]);
    out[3][row+2] = 2. * (quat[1] * reference[0] + quat[2] * reference[1] + quat[3] * reference[2]);
}
