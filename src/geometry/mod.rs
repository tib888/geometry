use na::base::dimension::{DimName, U2, U3};
use na::{IsometryMatrix2, IsometryMatrix3, Point, Unit, VectorN};

pub mod circle;
pub mod line;

//to get rid of some genericness choose meaningful defaults for now:
pub type Length = f64;

//?pub struct Angle360(f64);

pub type Parameter = f64;
//?pub type Parameter2 = Point<Parameter, U2>;

pub type Position<D> = Point<Length, D>;
pub type Vector<D> = VectorN<Length, D>;
pub type Direction<D> = Unit<VectorN<f64, D>>; //f32?

pub type Position2 = Position<U2>;
pub type Vector2 = Vector<U2>;
pub type Direction2 = Direction<U2>;

pub type Position3 = Position<U3>;
pub type Vector3 = Vector<U3>;
pub type Direction3 = Direction<U3>;

pub type Transformation2 = IsometryMatrix2<Length>;
pub type Transformation3 = IsometryMatrix3<Length>;

const DIV_OVERFLOW: f64 = 1E-15;

pub trait SameInEpsilon {
    fn same_in_epsilon(a: &Self, b: &Self, epsilon: f64) -> bool;
}

impl SameInEpsilon for Length {
    fn same_in_epsilon(a: &Self, b: &Self, epsilon: f64) -> bool {
        (a - b).abs() <= epsilon
    }
}

impl<D> SameInEpsilon for Position<D>
where
    D: DimName,
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    fn same_in_epsilon(a: &Self, b: &Self, epsilon: f64) -> bool {
        na::distance(a, b) <= epsilon
    }
}

impl<D> SameInEpsilon for Vector<D>
where
    D: DimName,
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    fn same_in_epsilon(a: &Self, b: &Self, epsilon: f64) -> bool {
        (a - b).norm() <= epsilon
    }
}

impl<D> SameInEpsilon for Direction<D>
where
    D: DimName,
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    fn same_in_epsilon(a: &Self, b: &Self, epsilon: f64) -> bool {
        Vector::<D>::same_in_epsilon(a.as_ref(), b.as_ref(), epsilon)
    }
}

pub fn rotate_cw90(dir: &Direction2) -> Direction2 {
    Unit::new_unchecked(Vector::<U2>::new(dir[1], -dir[0]))
}

pub fn rotate_ccw90(dir: &Direction2) -> Direction2 {
    Unit::new_unchecked(Vector::<U2>::new(-dir[1], dir[0]))
}

pub fn perpendicular(dir: &Direction3) -> Direction3 {
    let v = dir.as_ref();
    let ax = if (v[0].abs() < 1f64 / 64f64) && (v[1].abs() < 1f64 / 64f64) {
        Vector3::y().cross(v)
    } else {
        Vector3::z().cross(v)
    };
    Direction3::new_normalize(ax)
}

pub trait UntrimmedProjector<D: DimName>
where
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    fn project(&self, p: &Position<D>) -> (Position<D>, Option<Direction<D>>);

    fn calculate_signed_distance(&self, p: &Position<D>) -> Length {
        let (projection, normal) = self.project(p);
        if let Some(n) = normal {
            (p - projection).dot(n.as_ref())
        } else {
            na::distance(p, &projection)
        }
    }
}

pub struct Range<T>
where
    T: na::Scalar,
{
    pub min: T,
    pub max: T,
}

impl<T> Range<T>
where
    T: na::Scalar,
{
    pub fn new(min: T, max: T) -> Self {
        Self { min, max }
    }
}

pub trait ParameterSpace {
    fn period(&self) -> Option<Parameter>;
    fn limits(&self) -> Option<Range<Parameter>>;
    fn closed(&self) -> bool;
}

pub trait ParametricCurve3 {
    fn all(&self, t: Parameter) -> (Position3, Vector3);
    fn xyz(&self, t: Parameter) -> Position3 {
        let (pos, _tangent) = self.all(t);
        pos
    }
    //fn t() -> T where T: ParameterSpace;
}

pub trait ParametricSurface3<U, V> {
    fn all(&self, u: Parameter, v: Parameter) -> (Position3, Vector3, Vector3, Direction3);
    fn xyz(&self, u: Parameter, v: Parameter) -> Position3 {
        let (pos, _du, _dv, _normal) = self.all(u, v);
        pos
    }
    //fn u() -> U where U: ParameterSpace;
    //fn v() -> V where V: ParameterSpace;
}

pub trait ProjectionTargetCurve: UntrimmedProjector<U3> + ParametricCurve3 {}

pub enum ParametricCurves {
    Ray(line::Ray3),
    Circle(circle::ParametricCircle),
}

pub fn to_dynamic(curve: ParametricCurves) -> Box<dyn ProjectionTargetCurve> {
    let curve: Box<dyn ProjectionTargetCurve> = match curve {
        ParametricCurves::Ray(ray) => Box::new(ray),
        ParametricCurves::Circle(circle) => Box::new(circle),
    };
    curve
}

// struct TrimmedCurve<T> {
//     curve: ParametricCurve3<T>,
//     t: ParameterSpace<T>,
// }

// struct Face<U, V> {
//     surface: ParametricSurface3<U, V>,
//     u: ParameterSpace<U>,
//     v: ParameterSpace<V>,
// }

// ///Lays in XY plane, axis is along Z
// pub struct CircleDefinition
// {
//     radius: Length,
// }

// ///Lays in XY plane, center is in the Origo
// pub struct EllipseDefinition
// {
//     radius_x: Length,
//     radius_y: Length,
// }

// pub struct PolyLine<D: DimName>
// {
//     points: Vec<Position<D>>;
// }

// pub enum CurveDefinitions2
// {
//     Circle: CircleDefinition,
//     Ellipse: EllipseDefinition,
// }

// pub struct Curve2
// {
//     definition: CurveDefinition2;
//     transformation: DirectIsometry2;
// }

// pub struct FlatCurve3
// {
//     definition: CurveDefinition2;
//     transformation: DirectIsometry3;
// }

// pub enum Curves3
// {
//     PolyLine: PolyLine<3>,
//     FlatCurve: FlatCurve3,
// }

// ///lays in XY plane
// pub struct PlaneDefinition
// {
// }

// ///axis is along Z
// pub struct CylinderDefinition
// {
//     radius: Length
// }


// pub struct FiniteCylinder
// {
//     definition: CylinderDefinition
//     length: Option<Length>
//     arc: Option<AngularRange360>
// }

