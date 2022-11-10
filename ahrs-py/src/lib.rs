use pyo3::prelude::*;

#[pyclass]
struct MargEkf {
    inner: ahrs::MargEkf,
}

#[pymethods]
impl MargEkf {
    #[new]
    fn new() -> MargEkf {
        MargEkf {
            inner: ahrs::MargEkf::new(),
        }
    }

    #[getter]
    fn state(&self) -> PyResult<[f32; 7]> {
        Ok(self.inner.state[0])
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
    m.add_class::<MargEkf>()?;
    Ok(())
}
