/**
	 * Holds a three degree of freedom orientation.
	 *
	 * Quaternions have
	 * several mathematical properties that make them useful for
	 * representing orientations, but require four items of data to
	 * hold the three degrees of freedom. These four items of data can
	 * be viewed as the coefficients of a complex number with three
	 * imaginary parts. The mathematics of the quaternion is then
	 * defined and is roughly correspondent to the math of 3D
	 * rotations. A quaternion is only a valid rotation if it is
	 * normalised: i.e. it has a length of 1.
	 *
	 * @note Angular velocity and acceleration can be correctly
	 * represented as vectors. Quaternions are only needed for
	 * orientation.
	 */

#include "MyConst.h"

class MyQuaternion
{
public:
	union {
		struct {
			/**
			 * Holds the double component of the quaternion.
			 */
			double r;

			/**
			 * Holds the first complex component of the
			 * quaternion.
			 */
			double i;

			/**
			 * Holds the second complex component of the
			 * quaternion.
			 */
			double j;

			/**
			 * Holds the third complex component of the
			 * quaternion.
			 */
			double k;
		};

		/**
		 * Holds the quaternion data in array form.
		 */
		double data[4];
	};

	// ... other MyQuaternion code as before ...

	/**
	 * The default constructor creates a quaternion representing
	 * a zero rotation.
	 */
	MyQuaternion() : r(1), i(0), j(0), k(0) {}

	/**
	 * The explicit constructor creates a quaternion with the given
	 * components.
	 *
	 * @param r The double component of the rigid body's orientation
	 * quaternion.
	 *
	 * @param i The first complex component of the rigid body's
	 * orientation quaternion.
	 *
	 * @param j The second complex component of the rigid body's
	 * orientation quaternion.
	 *
	 * @param k The third complex component of the rigid body's
	 * orientation quaternion.
	 *
	 * @note The given orientation does not need to be normalised,
	 * and can be zero. This function will not alter the given
	 * values, or normalise the quaternion. To normalise the
	 * quaternion (and make a zero quaternion a legal rotation),
	 * use the normalise function.
	 *
	 * @see normalise
	 */
	MyQuaternion(const double r, const double i, const double j, const double k)
		: r(r), i(i), j(j), k(k)
	{
	}

	/**
	 * Normalises the quaternion to unit length, making it a valid
	 * orientation quaternion.
	 */
	void normalise()
	{
		double d = r * r + i * i + j * j + k * k;

		// Check for zero length quaternion, and use the no-rotation
		// quaternion in that case.
		if (d < real_epsilon) {
			r = 1;
			return;
		}

		d = ((double)1.0) / real_sqrt(d);
		r *= d;
		i *= d;
		j *= d;
		k *= d;
	}

	/**
	 * Multiplies the quaternion by the given quaternion.
	 *
	 * @param multiplier The quaternion by which to multiply.
	 */
	void operator *=(const MyQuaternion &multiplier)
	{
		MyQuaternion q = *this;
		r = q.r*multiplier.r - q.i*multiplier.i -
			q.j*multiplier.j - q.k*multiplier.k;
		i = q.r*multiplier.i + q.i*multiplier.r +
			q.j*multiplier.k - q.k*multiplier.j;
		j = q.r*multiplier.j + q.j*multiplier.r +
			q.k*multiplier.i - q.i*multiplier.k;
		k = q.r*multiplier.k + q.k*multiplier.r +
			q.i*multiplier.j - q.j*multiplier.i;
	}

	/**
	 * Adds the given vector to this, scaled by the given amount.
	 * This is used to update the orientation quaternion by a rotation
	 * and time.
	 *
	 * @param vector The vector to add.
	 *
	 * @param scale The amount of the vector to add.
	 */
	void addScaledVector(const glm::vec3& vector, double scale)
	{
		MyQuaternion q(0,
			vector.x * scale,
			vector.y * scale,
			vector.z * scale);
		q *= *this;
		r += q.r * ((double)0.5);
		i += q.i * ((double)0.5);
		j += q.j * ((double)0.5);
		k += q.k * ((double)0.5);
	}

	void rotateByVector(const glm::vec3& vector)
	{
		MyQuaternion q(0, vector.x, vector.y, vector.z);
		(*this) *= q;
	}
};