pub fn invert<const I: usize>(mut m: [[f32; I]; I]) -> [[f32; I]; I] {
    /*
    let mut augmented: Array<f32, Ix2> = Array::zeros((n, 2 * n));
    augmented.slice_mut(s![0..n, 0..n]).assign(&(matrix));
    augmented
        .slice_mut(s![0..n, n..2 * n])
        .assign(&Array::eye(n));
    */
    let mut o = eye::<I>(1.);
    /*
    for i in 0..n {
        assert_ne!(augmented[[i, i]], 0.0); // TODO handle not invertable
        for j in 0..n {
            if i != j {
                let ratio = augmented[[j, i]] / augmented[[i, i]];
                for k in 0..2 * n {
                    augmented[[j, k]] = augmented[[j, k]] - ratio * augmented[[i, k]];
                }
            }
        }
    }
    */
    for i in 0..I {
        assert_ne!(m[i][i], 0.);
        for j in 0..I {
            if i != j {
                let ratio = m[i][j] / m[i][i];
                for k in 0..I {
                    m[k][j] = m[k][j] - ratio * m[k][i];
                    o[k][j] = o[k][j] - ratio * o[k][i];
                }
            }
        }
    }

    /*
    for i in 0..n {
        for j in n..2 * n {
            augmented[[i, j]] = augmented[[i, j]] / augmented[[i, i]];
        }
    }
    */

    for i in 0..I {
        for j in 0..I {
            o[j][i] = o[j][i] / m[i][i];
        }
    }
    /*
    let mut ret: Array<f32, Ix2> = Array::zeros((n, n));
    ret.slice_mut(s![0..n, 0..n])
        .assign(&augmented.slice(s![0..n, n..2 * n]));
    ret
     */

    o
}

#[test]
pub fn test_matrix_invert() {
    let test = [[1., 2., 3.], [0., 1., 4.], [5., 6., 0.]];
    let expected = [[-24., 18., 5.], [20., -15., -4.], [-5., 4., 1.]];
    assert_eq!(invert(test), expected);
}

pub fn eye<const I: usize>(v: f32) -> [[f32; I]; I] {
    let mut m: [[f32; I]; I] = [[0.; I]; I];
    for i in 0..I {
        m[i][i] = v;
    }
    m
}

pub fn mset<const I: usize, const J: usize>(o: &mut [[f32; I]; J], v: f32) {
    for i in 0..I {
        for j in 0..J {
            o[j][i] = v;
        }
    }
}

// o_{I×K} += l_{I×J} × r_{J×K}
pub fn mmulm_inc<const I: usize, const J: usize, const K: usize>(l: &[[f32; I]; J], r: &[[f32; J]; K], o: &mut [[f32; I]; K]) {
    for i in 0..I {
        for j in 0..J {
            for k in 0..K {
                o[k][i] += l[j][i] * r[k][j];
            }
        }
    }
}

// o_{I×K} += l_{I×J} × r_{K×J}^T
pub fn mmult_inc<const I: usize, const J: usize, const K: usize>(l: &[[f32; I]; J], r: &[[f32; K]; J], o: &mut [[f32; I]; K]) {
    for i in 0..I {
        for j in 0..J {
            for k in 0..K {
                o[k][i] += l[j][i] * r[j][k];
            }
        }
    }
}

// o_{I×K} += l_{J×I}^T × r_{J×K}
pub fn tmulm_inc<const I: usize, const J: usize, const K: usize>(l: &[[f32; J]; I], r: &[[f32; J]; K], o: &mut [[f32; I]; K]) {
    for i in 0..I {
        for j in 0..J {
            for k in 0..K {
                o[k][i] += l[i][j] * r[k][j];
            }
        }
    }
}

pub fn mmulm<const I: usize, const J: usize, const K: usize>(l: &[[f32; I]; J], r: &[[f32; J]; K]) -> [[f32; I]; K] {
    let mut out = [[0.; I]; K];
    mmulm_inc(l, r, &mut out);
    out
}

pub fn mmult<const I: usize, const J: usize, const K: usize>(l: &[[f32; I]; J], r: &[[f32; K]; J]) -> [[f32; I]; K] {
    let mut out = [[0.; I]; K];
    mmult_inc(l, r, &mut out);
    out
}

pub fn msubm<const I: usize, const J: usize>(l: &[[f32; I]; J], r: &[[f32; I]; J]) -> [[f32; I]; J] {
    let mut o = [[0.; I]; J];
    for i in 0..I {
        for j in 0..J {
            o[j][i] = l[j][i] - r[j][i];
        }
    }
    o
}

pub fn flip<const I: usize, const J: usize>(m: [[f32; I]; J]) -> [[f32; J]; I] {
    let mut o = [[0.; J]; I];
    for i in 0..I {
        for j in 0..J {
            o[i][j] = m[j][i];
        }
    }
    o
}

#[test]
fn test_mmul() {
    let mut a = eye::<5>(1.);
    a[1][0] = 1.;
    a[1][2] = 1.;
    let b = [[1., 2., 3., 5., 7.]];
    let mut o = [[0.; 5]];
    mmulm_inc(&a, &b, &mut o);
    assert_eq!(o, [[3., 2., 5., 5., 7.]]);
}

pub trait Sqrt {
    fn sqrt(&self) -> Self;
}

impl Sqrt for f32 {
    fn sqrt(&self) -> f32 {
        libm::sqrtf(*self)
    }
}
