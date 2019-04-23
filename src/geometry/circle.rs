use super::*;

pub trait Circle<D: DimName>
where
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    fn radius(&self) -> Length;
    fn center(&self) -> Position<D>;
    fn axis_direction(&self) -> Direction3;
}

// pub trait CircleArc<D: DimName>: Circle<D>
// where
//     na::DefaultAllocator: na::allocator::Allocator<f64, D>,
// {
//     fn start(&self) -> Angle360;
//     fn end(&self) -> Angle360;
// }

pub struct Circle2 {
    radius: Length,
    center: Position2,
}

impl Circle<U2> for Circle2 {
    fn radius(&self) -> Length {
        self.radius
    }
    fn center(&self) -> Position2 {
        self.center
    }
    fn axis_direction(&self) -> Direction3 {
        Direction3::new_unchecked(Vector3::z())
    }
}

impl Circle2 {
    pub fn new(radius: Length, center: Position2) -> Self {
        Circle2 { radius, center }
    }
}


impl UntrimmedProjector<U2> for Circle<U2> {
    fn project(&self, p: &Position<U2>) -> (Position<U2>, Option<Direction<U2>>) {
        let n = if let Some(n) = Direction2::try_new(p - self.center(), DIV_OVERFLOW) {
            n
        } else {
            //if p is in the circle center, choose arbitrary direction (1, 0)
            Direction2::new_unchecked(Vector2::x())
        };
        (self.center() + n.as_ref() * self.radius(), Some(n))
    }

    fn calculate_signed_distance(&self, p: &Position<U2>) -> Length {
        (p - self.center()).norm() - self.radius()
    }
}

pub struct Circle3 {
    radius: Length,
    center: Position3,
    axis_direction: Direction3,
}

impl Circle<U3> for Circle3 {
    fn radius(&self) -> Length {
        self.radius
    }
    fn center(&self) -> Position3 {
        self.center
    }
    fn axis_direction(&self) -> Direction3 {
        self.axis_direction
    }
}

impl Circle3 {
    pub fn new(radius: Length, center: Position3, axis_direction: Direction3) -> Self {
        Circle3 {
            radius,
            center,
            axis_direction,
        }
    }
}

pub struct ParametricCircle {
    tr: Transformation3,
    radius: Length,
}

impl Circle<U3> for ParametricCircle {
    fn radius(&self) -> Length {
        self.radius
    }
    fn center(&self) -> Position3 {
        Position3::from(self.tr.translation.vector)
    }
    fn axis_direction(&self) -> Direction3 {
        Direction3::new_unchecked(Vector3::from(self.tr.rotation.matrix().column(2)))
    }
}

impl ParametricCurve3 for ParametricCircle {
    fn all(&self, t: Parameter) -> (Position3, Vector3) {
        let c = t.cos() * self.radius;
        let s = t.sin() * self.radius;
        let pos = Position3::new(c, s, 0f64);
        let tangent = Vector3::new(s, c, 0f64);
        (
            self.tr.transform_point(&pos),
            self.tr.transform_vector(&tangent),
        )
    }
}

impl ParameterSpace for ParametricCircle {
    fn period(&self) -> Option<Parameter> {
        Some(std::f64::consts::PI + std::f64::consts::PI)
    }
    fn limits(&self) -> Option<Range<Parameter>> {
        None //? Some(Range::new(0f64, std::f64::consts::PI + std::f64::consts::PI))
    }
    fn closed(&self) -> bool {
        true
    }
}

impl<T> UntrimmedProjector<U3> for T
where
    T: Circle<U3>,
{
    fn project(&self, p: &Position<U3>) -> (Position3, Option<Direction3>) {
        let v0 = p - self.center();
        let v1 = v0 - self.axis_direction().as_ref() * v0.dot(self.axis_direction().as_ref());
        let n = if let Some(n) = Direction3::try_new(v1, DIV_OVERFLOW) {
            n
        } else {
            //if p is in the circle center, choose arbitrary direction
            perpendicular(&self.axis_direction())
        };
        (self.center() + n.as_ref() * self.radius(), None)
    }
}

impl ProjectionTargetCurve for ParametricCircle {}
