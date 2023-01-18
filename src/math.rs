pub type Vec2 = glm::DVec2;
pub type Vec3 = glm::DVec3;
pub type Vec4 = glm::DVec4;
pub type Mat3 = glm::DMat3;
pub type Mat4 = glm::DMat4;

pub type Mat2x3 = glm::DMat2x3;
pub type Mat3x2 = glm::DMat3x2;
pub type Mat3x4 = glm::DMat3x4;

pub type Qua = glm::Qua<f64>;

pub fn v1_of_mat3(m: &Mat3) -> Vec3 {
    Vec3::new(m.m11, m.m21, m.m31)
}

pub fn v2_of_mat3(m: &Mat3) -> Vec3 {
    Vec3::new(m.m12, m.m22, m.m32)
}

pub fn v3_of_mat3(m: &Mat3) -> Vec3 {
    Vec3::new(m.m13, m.m23, m.m33)
}

pub fn make_mat2x3_from_vec3(v1: Vec3, v2: Vec3) -> Mat2x3 {
    Mat2x3::new(v1.x, v2.x, v1.y, v2.y, v1.z, v2.z)
}

pub fn make_mat3x2_from_vec3(v1: Vec3, v2: Vec3) -> Mat3x2 {
    Mat3x2::new(v1.x, v2.x, v1.y, v2.y, v1.z, v2.z)
}

pub fn make_mat3_from_vec3(v1: Vec3, v2: Vec3, v3: Vec3) -> Mat3 {
    Mat3::new(v1.x, v2.x, v3.x, v1.y, v2.y, v3.y, v1.z, v2.z, v3.z)
}

pub fn make_mat3x4_from_vec3(v1: Vec3, v2: Vec3, v3: Vec3, v4: Vec3) -> Mat3x4 {
    Mat3x4::new(v1.x, v2.x, v3.x, v4.x, v1.y, v2.y, v3.y, v4.y, v1.z, v2.z, v3.z, v4.z)
}

pub fn rotation_mat4(angle: Vec3) -> Mat4 {
    glm::rotation(angle.magnitude(), &angle)
}

pub fn rotation_mat3(angle: Vec3) -> Mat3 {
    glm::mat4_to_mat3(&rotation_mat4(angle))
}

pub fn make_transformation_matrix(l: Vec3, r: Mat3) -> Mat4 {
    Mat4::new(r.m11, r.m12, r.m13, l.x, r.m21, r.m22, r.m23, l.y, r.m31, r.m32, r.m33, l.z, 0.0, 0.0, 0.0, 1.0)
}

pub fn axis_and_angle_of_rotation_matrix(m: &Mat3) -> (Vec3, f64) {
    /*
    Assume the rotation matrix
    ┌       ┐
    │ a b c │
    │ d e f │
    │ g h i │
    └       ┘
    represents the rotation around the vector u by θ, then
    ┌       ┐   ┌          ┐
    │ h - f │   │ u_x2sinθ │
    │ c - g │ = │ u_y2sinθ │
    │ d - b │   │ u_z2sinθ │
    └       ┘   └          ┘
     */
    let u2sinθ: Vec3 = Vec3::new(m.m32-m.m23, m.m13-m.m31, m.m21-m.m12);
    let u: Vec3 = u2sinθ.normalize();
    // a = cosθ + u_x^2(1 - cosθ) => cosθ = (a - u_x^2) / (1 - u_x^2)
    let u_x_2 = u.x * u.x;
    (u, glm::acos(&glm::DVec1::new((m.m11 - u_x_2) / (1.0 - u_x_2))).magnitude())
}

pub fn angle_of_rotation_matrix(m: &Mat3) -> Vec3 {
    let u2sinθ: Vec3 = Vec3::new(m.m32-m.m23, m.m13-m.m31, m.m21-m.m12);
    let u: Vec3 = u2sinθ.normalize();
    let u_x_2 = u.x * u.x;
    u * (glm::acos(&glm::DVec1::new((m.m11 - u_x_2) / (1.0 - u_x_2))).magnitude())
}

pub fn reduced_row_echelon_form(m: &Mat3) {
    fn sort_rows(d: &mut [[f64; 3]; 3], l: (usize, usize)) -> bool {
        let mut col_is_zero = true;
        if d[l.0][l.1] == 0.0 {
            for i in l.0..3 {
                if d[i][l.1] != 0.0 {
                    let row = d[l.0];
                    d[l.0] = d[i];
                    d[i] = row;
                    col_is_zero = false;
                    return col_is_zero;
                }
            }
        }
        col_is_zero
    }
    let mut d = [
        [m.m11, m.m12, m.m13],
        [m.m21, m.m22, m.m23],
        [m.m31, m.m32, m.m33],
    ];
    let mut l: (usize, usize) = (0, 0);
    loop {
        if d[l.0][l.1] == 0.0 {
            for i in l.0..3 {
                if d[i][l.1] != 0.0 {}
            }
        }
    }
    /* let mut col_0_is_zero = false;
    if d[0][0] == 0.0 {
        if d[1][0] == 0.0{
            if d[2][0] == 0.0 {
                col_0_is_zero = true
            } else {
                let row = d[0];
                d[0] = d[2];
                d[2] = row;
            }
        } else {
            let row = d[0];
            d[0] = d[1];
            d[1] = row;
        }
    }
    if col_0_is_zero {

    } else {
        let factor = 1.0 / d[0][0];
        d[0][1] = d[0][1]
    } */
}

/* pub fn triangle_intersection(v1: Vec3, v2: Vec3, v3: Vec3, o: Vec3, d: Vec3) {
    let e1: Vec3 = v2 - v1;
    let e2: Vec3 = v3 - v1;
    let p: Vec3 = d.cross(&e2);
} */

// direction normalized
pub fn intersection(o: Vec3, d: Vec3, p: Vec3, normal: Vec3) -> Vec3 {
    o + d * ((p - o).dot(&normal) / d.dot(&normal))
}

// (0, 0), (0.5, 1), (1, 0)
pub fn parabola(x: f64) -> f64 {
    4.0 * x * (1.0 - x)
}

enum Operations {
    Add(f64),
    Sub(f64),
    Mul(f64),
    Div(f64),
    Powf(f64),
    Powi(i32),
    Exp(f64),
    Sqrt(f64),
    Log(f64, f64),
    Ln(f64),
    Log10(f64),
    Log2(f64),
    Abs(f64),
}