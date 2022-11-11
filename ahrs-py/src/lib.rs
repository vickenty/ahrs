use pyo3::prelude::*;

#[pyclass]
struct MargEkfFork {
    inner: ahrs_fork::MargEkf,
}

#[pymethods]
impl MargEkfFork {
    #[new]
    fn new() -> MargEkfFork {
        MargEkfFork {
            inner: ahrs_fork::MargEkf::new(),
        }
    }

    #[getter]
    fn state(&self) -> PyResult<[f32; 7]> {
        Ok(self.inner.state[0])
    }

    #[getter]
    fn predicted_accel_mag(&self) -> PyResult<[f32; 6]> {
        Ok(self.inner.predicted_accel_mag[0])
    }

    fn predict(&mut self, w_x: f32, w_y: f32, w_z: f32, dt: f32) -> PyResult<()> {
        self.inner.predict(w_x, w_y, w_z, dt);
        Ok(())
    }

    fn update(&mut self, accel: [f32; 3], mag: [f32; 3]) -> PyResult<()> {
        self.inner.update(accel, mag);
        Ok(())
    }
}

#[pyclass]
struct MargEkfBofh {
    inner: ahrs_bofh::MargEkf,
}

#[pymethods]
impl MargEkfBofh {
    #[new]
    fn new() -> MargEkfBofh {
        MargEkfBofh {
            inner: ahrs_bofh::MargEkf::new(),
        }
    }

    #[getter]
    fn state(&self) -> PyResult<Vec<f32>> {
        Ok(self.inner.state.to_vec())
    }

    #[getter]
    fn predicted_accel_mag(&self) -> PyResult<Vec<f32>> {
        Ok(self.inner.predicted_accel_mag.to_vec())
    }

    fn predict(&mut self, w_x: f32, w_y: f32, w_z: f32, dt: f32) -> PyResult<()> {
        self.inner.predict(w_x, w_y, w_z, dt);
        Ok(())
    }

    fn update(&mut self, accel: [f32; 3], mag: [f32; 3]) -> PyResult<()> {
        self.inner.update(accel, mag);
        Ok(())
    }
}


#[pymodule]
fn ahrs_py(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<MargEkfFork>()?;
    m.add_class::<MargEkfBofh>()?;
    Ok(())
}
