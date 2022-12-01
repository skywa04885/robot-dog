#include "registrator.h"

namespace app::slam::point_cloud
{
  registrator_builder::registrator_builder()
  {}


  std::valarray<double> registrator::execute_compute_partial_derivatives(
    std::valarray<double> &parameters,
    boost::container::vector<point> &source,
    tree &target
  )
  {

    boost::container::vector <value> nearest;
    nearest.reserve(source.size());

    // Gets all the nearest points and pits them in the nearest vector.
    nearest.clear();
    for (const point &sourcePoint: source)
    {
      target.query(
        boost::geometry::index::nearest(sourcePoint, 1),
        std::back_inserter(nearest));
    }
    BOOST_ASSERT(nearest.size() == source.size());

    // Calculates the cosine and sine of the angle and stores them
    //  in variables to prevent unnecessary computations. Also computes
    //  the double variant of the source size.
    const double cos_of_angle = std::cos(parameters[2]);
    const double sin_of_angle = std::sin(parameters[2]);
    const auto n = boost::numeric_cast<double>(source.size());

    // Creates the value arrays for the points.
    static std::valarray<double> target_point_i(source.size()),
      target_point_j(source.size()),
      source_point_i(source.size()),
      source_point_j(source.size());

    // Creates the value arrays for the terms.
    static std::valarray<double> sum_of_squared_differences_term_a(source.size()),
      sum_of_squared_differences_term_b(source.size()),
      partial_with_respect_to_angle_term_a(source.size()),
      partial_with_respect_to_angle_term_b(source.size());

    // Initializes all the value arrays to prepare them for computation.
    for (size_t i = 0; i < source.size(); ++i)
    {
      const value &targetValue = nearest.at(i);
      const point &targetPoint = targetValue.first;

      target_point_i[i] = targetPoint.get<0>();
      target_point_j[i] = targetPoint.get<1>();

      const point &sourcePoint = source.at(i);

      source_point_i[i] = sourcePoint.get<0>();
      source_point_j[i] = sourcePoint.get<1>();
    }

    // Calculates two terms we call sum_of_squared_differences_term_a and sum_of_squared_differences_term_b. These two terms will be shared
    //  across all the final partial derivatives.

    // 1. Assigns the starting values to sum_of_squared_differences_term_a and sum_of_squared_differences_term_b.
    sum_of_squared_differences_term_a = target_point_i;
    sum_of_squared_differences_term_b = target_point_j;

    // 2. Subtract the i offsets from the i scalars of the target points.
    sum_of_squared_differences_term_a -= parameters[0];
    // 3. Subtract the source i scalars multiplied by the cosine of the angle.
    sum_of_squared_differences_term_a -= source_point_i * cos_of_angle;
    // 4. Subtract the source j scalars multiplied by the sine of the angle.
    sum_of_squared_differences_term_a -= source_point_j * sin_of_angle;

    // 5. Subtract the j offsets from the j scalars of the target points.
    sum_of_squared_differences_term_b -= parameters[1];
    // 6. Add the source i scalars multiplied by the sine of the angle.
    sum_of_squared_differences_term_b += source_point_i * sin_of_angle;
    // 7. Subtract the source j scalars multiplied by the cosine of the angle.
    sum_of_squared_differences_term_b -= source_point_j * cos_of_angle;

    // Calculates the terms partial_with_respect_to_angle_term_a and partial_with_respect_to_angle_term_b which both are used in the partial derivative
    //  for the angle.

    // 1. Give partial_with_respect_to_angle_term_a the value of the source i scalar multiplied by the sine of the angle.
    partial_with_respect_to_angle_term_a = source_point_i * sin_of_angle;
    // 2. Subtracts the source j scalar multiplied by the cosine of the angle from partial_with_respect_to_angle_term_a.
    partial_with_respect_to_angle_term_a -= source_point_j * cos_of_angle;

    // 3. Give partial_with_respect_to_angle_term_b the value of the source i scalar multiplied by the cosine of the angle.
    partial_with_respect_to_angle_term_b = source_point_i * cos_of_angle;
    // 4. Add the source j scalar multiplied by the sine of the angle to partial_with_respect_to_angle_term_b.
    partial_with_respect_to_angle_term_b += source_point_j * sin_of_angle;

    // Calculates the common denominator for all the partial derivatives.
    const double common_denominator = std::sqrt((sum_of_squared_differences_term_a * sum_of_squared_differences_term_a +
                                                 sum_of_squared_differences_term_b *
                                                 sum_of_squared_differences_term_b).sum() / n);

    // Calculates all the partial derivatives.
    std::valarray<double> result(3);
    result[0] = (-sum_of_squared_differences_term_a.sum()) / n;
    result[1] = (-sum_of_squared_differences_term_b.sum()) / n;
    result[2] = (sum_of_squared_differences_term_a * partial_with_respect_to_angle_term_a +
                 sum_of_squared_differences_term_b * partial_with_respect_to_angle_term_b).sum();

    // Divides the result by the common denominator.
    result /= common_denominator;

    // Returns the result.
    return boost::move(result);
  }

  registrator::execute_result registrator::execute(
    boost::container::vector<point> &source,
    tree &target)
  {
    std::valarray<double> learning_rate(0.0, 3);
    learning_rate[0] = 0.1;
    learning_rate[1] = 0.1;
    learning_rate[2] = 0.0005;
    std::valarray<double> parameters(0.0, 3);

    for (size_t i = 0; i < 100; ++i)
    {
      // Computes the partial derivatives based on the current parameters.
      std::valarray<double> partial_derivatives =
        execute_compute_partial_derivatives(parameters,
                                            source, target);

      // Performs some logging.
      std::cout<< "registrator [" << i << "] -> partial derivatives: "
                << round_to<double>(partial_derivatives[0], 9) << ", "
                << round_to<double>(partial_derivatives[1], 9) << ", "
                << round_to<double>(partial_derivatives[2], 9) << ", parameters: "
                << round_to<double>(parameters[0], 3) << "mm, "
                << round_to<double>(parameters[1], 3) << "mm, "
                << round_to<double>((parameters[2] / M_PI) * 180.0, 3) << "deg"
                << std::endl;

      // Updates the parameters.
      parameters -= partial_derivatives * learning_rate;

    }

    boost::array<double, 2> offsets{0.0, 0.0};
    double angle = 0.0;

    return {
      offsets,
      angle
    };
  }
} // namespace app::slam::point_cloud
