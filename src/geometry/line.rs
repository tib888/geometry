use super::*;

pub trait Line<D: DimName>
where
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    fn point(&self) -> &Position<D>;
    fn direction(&self) -> &Direction<D>;
}

pub struct Ray<D: DimName>
where
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    point: Position<D>,
    direction: Direction<D>,
}

pub type Ray2 = Ray<U2>;
pub type Ray3 = Ray<U3>;

impl<D> Line<D> for Ray<D>
where
    D: DimName,
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    fn point(&self) -> &Position<D> {
        &self.point
    }

    fn direction(&self) -> &Direction<D> {
        &self.direction
    }
}

impl<D> Ray<D>
where
    D: DimName,
    na::DefaultAllocator: na::allocator::Allocator<f64, D>,
{
    pub fn new(point: Position<D>, direction: Direction<D>) -> Self {
        Ray::<D> { point, direction }
    }
}

impl UntrimmedProjector<U3> for Ray<U3>
where
    na::DefaultAllocator: na::allocator::Allocator<f64, U3>,
{
    fn project(&self, p: &Position<U3>) -> (Position<U3>, Option<Direction<U3>>) {
        (
            self.point + self.direction.as_ref() * (p - self.point).dot(self.direction.as_ref()),
            None,
        )
    }
}

impl UntrimmedProjector<U2> for Ray<U2>
where
    na::DefaultAllocator: na::allocator::Allocator<f64, U2>,
{
    fn project(&self, p: &Position<U2>) -> (Position<U2>, Option<Direction<U2>>) {
        let &d = self.direction.as_ref();
        let n = Unit::new_normalize(Vector::<U2>::new(d[1], -d[0]));
        (
            self.point + self.direction.as_ref() * (p - self.point).dot(self.direction.as_ref()),
            Some(n),
        )
    }

    fn calculate_signed_distance(&self, p: &Position<U2>) -> Length {
        (p - &self.point).dot(rotate_cw90(&self.direction).as_ref())
    }
}

impl ParametricCurve3 for Ray3 {
    fn all(&self, t: Parameter) -> (Position3, Vector3) {
        (
            self.point + self.direction.as_ref() * t,
            self.direction.into_inner(),
        )
    }
}

impl ParameterSpace for Ray3 {
    fn period(&self) -> Option<Parameter> {
        None
    }
    fn limits(&self) -> Option<Range<Parameter>> {
        None
    }
    fn closed(&self) -> bool {
        false
    }
}

impl ProjectionTargetCurve for Ray3 {}

/*
pub struct LineSegment<D: DimName>
{
    start_point: Position<D>,
    end_point: Position<D>,
}

type LineSegment2 = LineSegment<U2>;
type LineSegment3 = LineSegment<U3>;

impl<D> Line<D> for Ray<D> where D: DimName
{
    fn point(&self) -> Position<D>
    {
        self.point
    }

    fn direction(&self) -> Direction<D>
    {
        self.end_point - self.start_point
    }
}

impl<D> LineSegment<D> where D: DimName
{
    fn new(start_point: Position<D>, end_point: Position<D>) -> Self
    {
        LineSegment::<D> { start_point, end_point }
    }
}

impl<D> From<LineSegment<D>> for Ray<D> where D: DimName {
    fn from(input: LineSegment<D>) -> Self {
        Ray::<D> { point: input.get_point(), direction: input.get_direction() }
    }
}
*/

#[cfg(test)]
mod tests {

    use super::super::*;
    use super::*;
    static EPSILON: f64 = 1E-15;

    #[test]
    fn test_project3() {
        let p = Position3::new(1f64, 1f64, 1f64);
        let line = Ray3::new(
            Position3::new(0f64, 0f64, 0f64),
            Direction3::new_normalize(Vector3::new(1f64, 1f64, 1f64)),
        );
        let (pp, n) = line.project(&p);
        assert_eq!(n, None);
        assert!(Position3::same_in_epsilon(&p, &pp, EPSILON));
    }

    #[test]
    fn test_project2() {
        let p = Position2::new(1f64, 1f64);
        let line = Ray2::new(
            Position2::new(0f64, 0f64),
            Direction2::new_normalize(Vector2::new(1f64, 1f64)),
        );
        let (pp, n) = line.project(&p);

        assert!(Position2::same_in_epsilon(&p, &pp, EPSILON));

        if let Some(normal) = n {
            assert!(Direction2::same_in_epsilon(
                &normal,
                &Direction2::new_normalize(Vector2::new(1f64, -1f64)),
                EPSILON
            ));
        } else {
            assert!(false);
        }
        assert!(Length::same_in_epsilon(
            &line.calculate_signed_distance(&Position2::new(1f64, 0f64)),
            &(2f64.sqrt() / 2f64),
            EPSILON
        ));
        assert!(Length::same_in_epsilon(
            &line.calculate_signed_distance(&Position2::new(0f64, 1f64)),
            &(2f64.sqrt() / -2f64),
            EPSILON
        ));
    }
}