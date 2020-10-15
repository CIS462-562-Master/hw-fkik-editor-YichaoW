#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen\Dense>

#pragma warning(disable:4018)
#pragma warning(disable:4244)

vec3 lerp(vec3 a, vec3 b, double u) {
	vec3 v = a * (1 - u) + b * u;
	return v;
}

// calculate shortest path
vec3 getShortestPath(vec3 a, vec3 b) {
	for (int i = 0; i < 3; i++) {
		a[i] = std::fmod(a[i], 360);
		b[i] = std::fmod(b[i], 360);
		if (abs(b[i] - a[i]) > 180) {
			if (b[i] > 0) {
				b[i] -= 360;
			}
			else if (b[i] < 0) {
				b[i] += 360;
			}
		}
	}
	return b;
}

ASplineVec3::ASplineVec3() : mInterpolator(new ABernsteinInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
    if (mInterpolator) delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
    mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
    return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
    mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
    return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
    double fps = getFramerate();

	if (mInterpolator) { delete mInterpolator; }
    switch (type)
    {
	case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
	case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
	case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break;
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
	case LINEAR_EULER: mInterpolator = new AEulerLinearInterpolatorVec3(); break;
	case CUBIC_EULER: mInterpolator = new AEulerCubicInterpolatorVec3(); break;
    };
    
    mInterpolator->setFramerate(fps);
    computeControlPoints();
    cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
    return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys[keyID].second = value;
    computeControlPoints();
    cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0)
    {
        mStartPoint = value;
        computeControlPoints(false);
    }
    else if (ID == mCtrlPoints.size() + 1)
    {
        mEndPoint = value;
		computeControlPoints(false);
    }
    else mCtrlPoints[ID-1] = value;
    cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
    mKeys.push_back(Key(time, value));

    if (updateCurve)
    {
        computeControlPoints();
        cacheCurve();
    }
}

int ASplineVec3::insertKey(double time, const vec3& value, bool updateCurve)
{
	if (mKeys.size() == 0)
	{
		appendKey(time, value, updateCurve);
		return 0;
	}

	for (int i = 0; i < mKeys.size(); ++i)
	{
		assert(time != mKeys[i].first);
		if (time < mKeys[i].first)
		{
			mKeys.insert(mKeys.begin() + i, Key(time, value));
			if (updateCurve)
			{
				computeControlPoints();
				cacheCurve();
			}
			return i;
		}
	}

	// Append at the end of the curve
	appendKey(time, value, updateCurve);
	return mKeys.size() - 1;
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
    if (mKeys.size() == 0)
    {
        appendKey(0, value, updateCurve);
    }
    else
    {
        double lastT = mKeys[mKeys.size() - 1].first;
        appendKey(lastT + 1, value, updateCurve);
    }
}

void ASplineVec3::deleteKey(int keyID)
{
    assert(keyID >= 0 && keyID < mKeys.size());
    mKeys.erase(mKeys.begin() + keyID);
    computeControlPoints();
    cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID) const
{
    assert(keyID >= 0 && keyID < mKeys.size());
    return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
    return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID) const
{
    assert(ID >= 0 && ID < mCtrlPoints.size()+2);
    if (ID == 0) return mStartPoint;
    else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
    else return mCtrlPoints[ID-1];
}

int ASplineVec3::getNumControlPoints() const
{
    return mCtrlPoints.size() + 2; // include endpoints
}

void ASplineVec3::clear()
{
    mKeys.clear();
}

double ASplineVec3::getDuration() const 
{
    return mKeys.size() == 0 ? 0 : mKeys[mKeys.size()-1].first;
}

double ASplineVec3::getNormalizedTime(double t) const 
{
    return (t / getDuration());
}

double ASplineVec3::getKeyTime(int keyID) const
{
	assert(keyID >= 0 && keyID < mKeys.size());
	return mKeys[keyID].first;
}

vec3 ASplineVec3::getValue(double t) const
{
    if (mCachedCurve.size() == 0 || mKeys.size() == 0) return vec3();
	if (t < mKeys[0].first)
		return mCachedCurve[0];
	else
		t -= mKeys[0].first;

    double dt = mInterpolator->getDeltaTime();
    int rawi = (int)(t / dt); // assumes uniform spacing
    double frac = (t - rawi*dt) / dt;

	int i = mLooping? rawi % mCachedCurve.size() : std::min<int>(rawi, mCachedCurve.size() - 1);
	int inext = mLooping ? (i + 1) % mCachedCurve.size() : std::min<int>(i + 1, mCachedCurve.size() - 1);

    vec3 v1 = mCachedCurve[i];
    vec3 v2 = mCachedCurve[inext];
    vec3 v = v1*(1 - frac) + v2 * frac;
    return v;
}

void ASplineVec3::cacheCurve()
{
    mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints(bool updateEndPoints)
{
	if (mKeys.size() >= 2 && updateEndPoints)
	{
		int totalPoints = mKeys.size();

		//If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
		//They lie on the tangent of the first and last interpolation points.
		vec3 tmp = mKeys[0].second - mKeys[1].second;
		double n = tmp.Length();
		mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

		tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
		n = tmp.Length();
		mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
	}
    mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

vec3* ASplineVec3::getCachedCurveData()
{
	return mCachedCurve.data();
}

vec3 * ASplineVec3::getControlPointsData()
{
	return mCtrlPoints.data();
}

int ASplineVec3::getNumCurveSegments() const
{
    return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
    return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
    mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
    return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
    return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys, 
    const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0;

	curve.clear();

	int numSegments = keys.size() - 1;
	for (int segment = 0; segment < numSegments; segment++)
    {
        for (double t = keys[segment].first; t < keys[segment+1].first - FLT_EPSILON; t += mDt)
        {
			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
			// u = 0.0 when t = keys[segment-1].first  
			// u = 1.0 when t = keys[segment].first
 
			u = (t - keys[segment].first) / (keys[segment + 1].first - keys[segment].first);

            val = interpolateSegment(keys, ctrlPoints, segment, u);
            curve.push_back(val);
        }
    }
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments - 1, u);
		curve.push_back(val);
	}
}


// Interpolate p0 and p1 so that t = 0 returns p0 and t = 1 returns p1
vec3 ALinearInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;

	// TODO: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1
	curveValue = lerp(key0, key1, u);
	return curveValue;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	curveValue = b0 * pow((1 - u), 3) + b1 * 3 * u * pow((1 - u), 2)
		+ b2 * 3 * pow(u, 2) * (1 - u) + b3 * pow(u, 3);

	return curveValue;
}

vec3 ACasteljauInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  deCsteljau alogithm
	vec3 b01, b11, b21, b02, b12;
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	b01 = lerp(b0, b1, u);
	b11 = lerp(b1, b2, u);
	b21 = lerp(b2, b3, u);
	b02 = lerp(b01, b11, u);
	b12 = lerp(b11, b21, u);
	curveValue = lerp(b02, b12, u);

	return curveValue;
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	Eigen::MatrixXd mG(1, 4);
	Eigen::MatrixXd mM(4, 4);
	Eigen::MatrixXd mU(4, 1);
	mM << 1, -3, 3, -1,
		0, 3, -6, 3,
		0, 0, 3, -3,
		0, 0, 0, 1;
	mU << 1, u, pow(u, 2), pow(u, 3);
	for (int i = 0; i < 3; i++) {
		mG << b0[i], b1[i], b2[i], b3[i];
		curveValue[i] = (mG * mM * mU)(0, 0);
	}
	return curveValue;
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 p0;
	vec3 p1;
	vec3 q0; // slope at p0
	vec3 q1; // slope at p1
	vec3 curveValue(0, 0, 0);

	// TODO: Compute the interpolated value h(u) using a cubic Hermite polynomial  
	curveValue = (2 * pow(u, 3) - 3 * pow(u, 2) + 1) * keys[segment].second
		+ (-2 * pow(u, 3) + 3 * pow(u, 2)) * keys[segment + 1].second
		+ (pow(u, 3) - 2 * pow(u, 2) + u) * ctrlPoints[segment]
		+ (pow(u, 3) - pow(u, 2)) * ctrlPoints[segment + 1];
	return curveValue;
}

vec3 ABSplineInterpolatorVec3::interpolateSegment(
    const std::vector<ASplineVec3::Key>& keys,
    const std::vector<vec3>& ctrlPoints, 
    int segment, double u)
{
	vec3 curveValue(0, 0, 0);

	// Hint: Create a recursive helper function N(knots,n,j,t) to calculate BSpline basis function values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = curve interval on knot vector in which to interpolate
	//     t = time value	

	// Step 1: determine the index j
	// Step 2: compute the n nonzero Bspline Basis functions N given j
	// Step 3: get the corresponding control points from the ctrlPoints vector
	// Step 4: compute the Bspline curveValue at time t

	return curveValue;
}

void ACubicInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys, 
    std::vector<vec3>& ctrlPoints, 
    vec3& startPoint, vec3& endPoint)
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;

    for (int i = 1; i < keys.size(); i++)
    {
        vec3 b0, b1, b2, b3;
		// TODO: compute b0, b1, b2, b3
		vec3 s0, s1;
		int index = i - 1;
		b0 = keys[index].second;
		b3 = keys[index + 1].second;

		if (index == 0) {
			s0 = (keys[index + 1].second - startPoint) / 2;
		} else {
			s0 = (keys[index + 1].second - keys[index - 1].second) / 2;
		}

		if (index == keys.size() - 2) {
			s1 = (endPoint - keys[index].second) / 2;
		} else {
			s1 = (keys[index + 2].second - keys[index].second) / 2;
		}

		b2 = b3 - s1 / 3;
		b1 = b0 + s0 / 3;
		
        ctrlPoints.push_back(b0);
        ctrlPoints.push_back(b1);
        ctrlPoints.push_back(b2);
        ctrlPoints.push_back(b3);
    }
}

#include <iostream>
void AHermiteInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints,
    vec3& startPoint, vec3& endPoint)
{
	ctrlPoints.clear();
	//ctrlPoints.resize(keys.size(), vec3(0, 0, 0));
	if (keys.size() <= 1) return;

	// TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
	// Step 2: Initialize D
	// Step 3: Solve AC=D for C
	// Step 4: Save control points in ctrlPoints
	Eigen::MatrixXd mA(keys.size(), keys.size()), mAInverse(keys.size(), keys.size());
	Eigen::Matrix<vec3, Eigen::Dynamic, 1> mD(keys.size(), 1);
	Eigen::Matrix<vec3, Eigen::Dynamic, 1> mC(keys.size(), 1);

	// initialize A
	for (int i = 0; i < keys.size(); i++) {
		mA(0, i) = 0;
		mA(keys.size() - 1, i) = 0;
	}
	mA(0, 0) = 2;
	mA(0, 1) = 1;
	mA(keys.size() - 1, keys.size() - 1) = 2;
	mA(keys.size() - 1, keys.size() - 2) = 1;
	for (int i = 1; i < keys.size() - 1; i++) {
		for (int j = 0; j < keys.size(); j++) {
			mA(i, j) = 0;
		}
		mA(i, i - 1) = 1;
		mA(i, i) = 4;
		mA(i, i + 1) = 1;
	}

	// initialize D
	mD(0, 0) = 3 * (keys[1].second - keys[0].second);
	for (int i = 1; i < keys.size() - 1; i++) {
		mD(i, 0) = 3 * (keys[i + 1].second - keys[i - 1].second);
	}
	mD(keys.size() - 1, 0) = 3 * (keys[keys.size() - 1].second - keys[keys.size() - 2].second);
	

	// calculate C
	mAInverse = mA.inverse();
	for (int i = 0; i < keys.size(); i++) {
		vec3 temp(0, 0, 0);
		for (int j = 0; j < keys.size(); j++) {
			temp += mAInverse(i, j) * mD(j, 0);
		}
		mC(i, 0) = temp;
	}

	for (int i = 0; i < keys.size(); i++) {
		ctrlPoints.push_back(mC(i, 0));
	}
}

double dN(std::vector<double> knots, int n, int j, double t, int l) {
	if (l != 0) {
		double c1 = dN(knots, n - 1, j, t, l - 1) / (knots[j + n] - knots[j]);
		double c2 = dN(knots, n - 1, j + 1, t, l - 1) / (knots[j + n + 1] - knots[j + 1]);
		return n * (c1 - c2);
	}
	else {
		 if (t < knots[j] || t > knots[j + n + 1]) {
				return 0;
		 }

		if (n == 0) {
			if (t >= knots[j] && t < knots[j + 1]) {
				return 1;
			}
			else {
				return 0;
			}
		}


		double c1 = (t - knots[j]) / (knots[j + n] - knots[j]) * dN(knots, n - 1, j, t, l);
		double c2 = (knots[j + n + 1] - t) / (knots[j + n + 1] - knots[j + 1]) * dN(knots, n - 1, j + 1, t, l);

		return c1 + c2;
	}
}

void ABSplineInterpolatorVec3::computeControlPoints(
    const std::vector<ASplineVec3::Key>& keys,
    std::vector<vec3>& ctrlPoints, 
    vec3& startPt, vec3& endPt)
{
    ctrlPoints.clear();
	//ctrlPoints.resize(keys.size() + 2, vec3(0, 0, 0));
    if (keys.size() <= 1) return;

	// TODO:
	// Hints: 
	// 1. use Eigen::MatrixXd to calculate the control points by solving the system of equations AC=D for C

	// 2. Create a recursive helper function dN(knots,n,t,l) to calculate derivative BSpline values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = interval on knot vector in which to interpolate
	//     t = time value
	//     l = derivative (l = 1 => 1st derivative)

	// Step 1: Calculate knot vector using a uniform BSpline
	//         (assune knots are evenly spaced 1 apart and the start knot is at time = 0.0)

	// Step 2: Calculate A matrix  for a natural BSpline
	//         (Set 2nd derivative at t0 and tm to zero, where tm is the last point knot; m = #segments)

	// Step 3: Calculate  D matrix composed of our target points to interpolate

	// Step 4: Solve AC=D for C 

	// Step 5: save control points in ctrlPoints
	// knots
	std::vector<double> knots;

	for (int i = 0; i < keys.size() + 6; i++) {
		knots.push_back(1.0 * (i-3));
	}

	// A
	Eigen::MatrixXd mA(keys.size() + 3, keys.size() + 3), mAInverse(keys.size() + 3, keys.size() + 3);
	for (int i = 0; i < keys.size() + 3; i++) {
		for (int j = 0; j < keys.size() + 3; j++) {
			mA(i, j) = 0;
		}
	}

	for (int i = 0; i < 4; i++) {
		mA(0, i) = dN(knots, 3, i, keys[0].first, 2);
		mA(0, mA.cols() - 1 - i) = dN(knots, 3, keys.size() + 2 - i, keys[keys.size() - 1].first, 2);
	}

	for (int i = 1; i < mA.rows() - 1; i++) {
		for (int j = 0; j < mA.cols()- 1; j++) {
			mA(i, j + i - 1) = dN(knots, 3, j, keys[i - 1].first, 0);
		
		}
	}
	
	// D
	Eigen::Matrix<vec3, Eigen::Dynamic, 1> mD(keys.size() + 3, 1);
	for (int i = 0; i < keys.size(); i++) {
		mD(i + 1, 0) = keys[i].second;
	}
	mD(0, 0) = 0;
	mD(mD.rows() - 1, 0) = 0;

	//std::cout << mD << "\n";

	// solve for c
	Eigen::Matrix<vec3, Eigen::Dynamic, 1> mC(keys.size() + 3, 1);
	mAInverse = mA.inverse();
	for (int i = 0; i < mC.rows(); i++) {
		vec3 temp;
		for (int j = 0; j < mAInverse.cols(); j++) {
			temp += mAInverse(i, j) * mD(j, 0);
		}
		mC(i, 0) = temp;
	}


	for (int i = 0; i < mC.rows(); i++) {
		ctrlPoints.push_back(mC(i, 0));
	}
}


vec3 AEulerLinearInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys, 
	const std::vector<vec3>& ctrlPoints, 
	int segment, double u)
{
	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;

	// TODO:
	// Linear interpolate between key0 and key1
	// You should convert the angles to find the shortest path for interpolation
	for (int i = 0; i < 3; i++) {
		key0[i] = fmod(key0[i], 360);
	}
	

	key1 = getShortestPath(key0, key1);
	curveValue = lerp(key0, key1, u);
	return curveValue;
}

vec3 AEulerCubicInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys, 
	const std::vector<vec3>& ctrlPoints, int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	// You should convert the angles to find the shortest path for interpolation
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	double u = t;
	curveValue = b0 * pow((1 - u), 3) + b1 * 3 * u * pow((1 - u), 2)
		+ b2 * 3 * pow(u, 2) * (1 - u) + b3 * pow(u, 3);

	return curveValue;
}

void AEulerCubicInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys, 
	std::vector<vec3>& ctrlPoints, vec3 & startPoint, vec3 & endPoint)
{
	ctrlPoints.clear();
	if (keys.size() <= 1) return;

	// Hint: One naive way is to first convert the keys such that the differences of the x, y, z Euler angles 
	//		 between every two adjacent keys are less than 180 degrees respectively 

	std::vector<vec3> shortestPathKeys;
	for (int i = 0; i < keys.size(); i++) {
		vec3 temp;
			
		if (i == 0) {
			temp = getShortestPath(startPoint, keys[0].second);
		}
		else {
			temp = getShortestPath(keys[i - 1].second, keys[i].second);
		}
		shortestPathKeys.push_back(temp);
	}
	for (int i = 1; i < keys.size(); i++)
	{
		vec3 b0, b1, b2, b3;

		// TODO: compute b0, b1, b2, b3
		vec3 s0, s1;
		int index = i - 1;

		for (int i = 0; i < 3; i++) {
			startPoint[i] = std::fmod(startPoint[i], 360);
			endPoint[i] = std::fmod(endPoint[i], 360);
		}
		
		if (index == 0) {
			s0 = (shortestPathKeys[index + 1] - startPoint) / 2;
		}
		else {
			s0 = (shortestPathKeys[index + 1] - shortestPathKeys[index - 1]) / 2;
		}

		if (index == keys.size() - 2) {
			s1 = (endPoint - shortestPathKeys[index]) / 2;
		}
		else {
			s1 = (shortestPathKeys[index + 2] - shortestPathKeys[index]) / 2;
		}

		b0 = shortestPathKeys[index];
		b3 = shortestPathKeys[index + 1];
		b2 = b3 - s1 / 3;
		b1 = b0 + s0 / 3;
		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
}
