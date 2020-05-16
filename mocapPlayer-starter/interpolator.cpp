#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
	//Set default interpolation type
	m_InterpolationType = LINEAR;

	//set default angle representation to use for interpolation
	m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N)
{
	//Allocate new motion
	*pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton());

	//Perform the interpolation
	if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
		LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
		LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
		BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
		BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else
	{
		printf("Error: unknown interpolation / angle representation type.\n");
		exit(1);
	}
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
				interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
	double cy = sqrt(R[0] * R[0] + R[3] * R[3]);

	if (cy > 16 * DBL_EPSILON)
	{
		angles[0] = atan2(R[7], R[8]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = atan2(R[3], R[0]);
	}
	else
	{
		angles[0] = atan2(-R[5], R[4]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = 0;
	}

	for (int i = 0; i < 3; i++)
		angles[i] *= 180 / M_PI;
}

double Degree2Radian(double degree){
    return degree / 180 * M_PI;
}

void MatrixMultiplication(double mat1[9], double mat2[9], double result[9])
{
  int i, j; 
    for (i = 0; i < 3; i++) 
    { 
        for (j = 0; j < 3; j++) 
        { 
            double sum = 0.0;
            for (int k = 0; k < 3; k++)
                sum = sum + mat1[i * 3 + k] * mat2[k * 3 + j];
            result[i * 3 + j] = sum;
        } 
    } 
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
	// students should implement this
	// Calculate rotation about x axis
	double theta1 = Degree2Radian(angles[0]);
	double theta2 = Degree2Radian(angles[1]);
	double theta3 = Degree2Radian(angles[2]);

	double R_x[9] = {
	  1,    0,              0,
	  0,    cos(theta1),    -sin(theta1),
	  0,    sin(theta1),    cos(theta1)
	};
	// Calculate rotation about y axis
	double R_y[9] = {
	  cos(theta2),   0,    sin(theta2),
	  0,             1,    0,
	  -sin(theta2),  0,    cos(theta2)
	};
	// Calculate rotation about z axis
	double R_z[9] = {
	  cos(theta3),    -sin(theta3),   0,
	  sin(theta3),    cos(theta3),    0,
	  0,              0,              1
	};
	// Combined rotation matrix   R_z * R_y * R_x
	double result[9];
	MatrixMultiplication(R_z, R_y, result);
	MatrixMultiplication(result, R_x, R);
}

double Interpolator::Degree2Radian(double degree) {
	return degree / 180 * M_PI;
}

void Interpolator::MatrixMultiplication(double mat1[9], double mat2[9], double result[9])
{
	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			double sum = 0.0;
			for (int k = 0; k < 3; k++)
				sum = sum + mat1[i * 3 + k] * mat2[k * 3 + j];
			result[i * 3 + j] = sum;
		}
	}
}

void ControlPointEuler(vector &p0, vector &p1, vector &p2, vector &p3, vector &a1, vector &b2, bool fp0, bool fp3) {
	double scaler = 1.0 / 3;
	if (fp0) {
		//first key frame
		a1 = p1 + (p2 * 2 - p3 - p1) * scaler;
		b2 = p2 - ((p2 * 2 - p1 + p3) * 0.5 - p2) * scaler;
	}
	else if (fp3) {
		//last key frame
		a1 = p1 + ((p1 * 2 - p0 + p2) * 0.5 - p1) * scaler;
		b2 = p2 + (p1 * 2 - p0 - p2) * scaler;
	}
	else {
		//inter frame
		a1 = p1 + ((p1 * 2 - p0 + p2) * 0.5 - p1) * scaler;
		b2 = p2 - ((p2 * 2 - p1 + p3) * 0.5 - p2) * scaler;
	}
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	// students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// key points
	vector p0, p1, p2, p3;
	// control points
	vector a1, b2;

	// bone orientation key points
	vector bp0, bp1, bp2, bp3;
	// bone orientation control points
	vector ba1, bb2;

	bool firstKF = true, lastKF = false, firstKFBone = true, lastKFBone = false;
	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int preKeyFrame = startKeyframe - N - 1;
		int endKeyframe = startKeyframe + N + 1;
		int nextKeyFrame = endKeyframe + N + 1;

		Posture * prePosture = NULL;
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		Posture * nextPosture = NULL;

		// copy start and end keyframe
		if (preKeyFrame >= 0)
			prePosture = pInputMotion->GetPosture(preKeyFrame);
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);
		if (nextKeyFrame < inputLength)
			nextPosture = pInputMotion->GetPosture(nextKeyFrame);

		//calculate p1, a1, b2, p2 for root interpolation
		p1 = startPosture->root_pos;
		p2 = endPosture->root_pos;

		if (startKeyframe == 0)
		{
			//first key frame
			firstKF = true;
			p3 = nextPosture->root_pos;
		}
		else if (nextKeyFrame > inputLength)
		{
			//last key frame
			lastKF = true;
			p0 = prePosture->root_pos;
		}
		else
		{
			//inter frame
			p0 = prePosture->root_pos;
			p3 = nextPosture->root_pos;
			firstKF = false;
			lastKF = false;
		}
		ControlPointEuler(p0, p1, p2, p3, a1, b2, firstKF, lastKF);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				bp1 = startPosture->bone_rotation[bone];
				bp2 = endPosture->bone_rotation[bone];

				if (startKeyframe == 0)
				{
					//first key frame
					firstKFBone = true;
					bp3 = nextPosture->bone_rotation[bone];
				}
				else if (nextKeyFrame > inputLength) {
					//last key frame
					lastKFBone = true;
					bp0 = prePosture->bone_rotation[bone];
				}
				else
				{
					//inter frame
					bp0 = prePosture->bone_rotation[bone];
					bp3 = nextPosture->bone_rotation[bone];
					firstKFBone = false;
					lastKFBone = false;
				}
				ControlPointEuler(bp0, bp1, bp2, bp3, ba1, bb2, firstKFBone, lastKFBone);
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, bp1, ba1, bb2, bp2);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	// students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	Quaternion<double> qStart, qEnd, qInter;

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);
				qInter = Slerp(t, qStart, qEnd);
				Quaternion2Euler(qInter, interpolatedPosture.bone_rotation[bone].p);
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::QuaternionControlPoint(Quaternion<double> &q0, Quaternion<double> &q1, Quaternion<double> &q2, Quaternion<double> &q3, Quaternion<double> &an, Quaternion<double> &bn, bool firstKF, bool lastKF) {
	double scaler = 1.0 / 3;
	Quaternion<double> a1;
	Quaternion<double> anp;
	Quaternion<double> bN;
	Quaternion<double> dual;

	if (firstKF)
	{
		//first key frame
		dual = Double(q3, q2);
		a1 = Slerp(scaler, q1, dual);
		an = a1;

		dual = Double(q1, q2);
		anp = Slerp(0.5, dual, q3);
		bn = Slerp(-scaler, q2, anp);
	}
	else if (lastKF) {
		//last key frame
		dual = Double(q0, q1);
		anp = Slerp(0.5, dual, q2);
		an = Slerp(scaler, q1, anp);

		dual = Double(q0, q1);
		bN = Slerp(scaler, q2, dual);
		bn = bN;
	}
	else
	{
		//inter frame
		dual = Double(q0, q1);
		anp = Slerp(0.5, dual, q2);
		an = Slerp(scaler, q1, anp);

		dual = Double(q1, q2);
		anp = Slerp(0.5, dual, q3);
		bn = Slerp(-scaler, q2, anp);
	}
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	// students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  // key points
	vector p0, p1, p2, p3;
	// control points
	vector a1, b2;

	// bone orientation key points
	Quaternion<double> q0, q1, q2, q3;
	// bone orientation control points
	Quaternion<double> an, bn, qInter;

	bool firstKF = true, lastKF = false, firstKFBone = true, lastKFBone = false;
	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int preKeyFrame = startKeyframe - N - 1;
		int endKeyframe = startKeyframe + N + 1;
		int nextKeyFrame = endKeyframe + N + 1;

		Posture * prePosture = NULL;
		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		Posture * nextPosture = NULL;

		// copy start and end keyframe
		if (preKeyFrame >= 0)
			prePosture = pInputMotion->GetPosture(preKeyFrame);
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);
		if (nextKeyFrame < inputLength)
			nextPosture = pInputMotion->GetPosture(nextKeyFrame);

		//calculate p1, a1, b2, p2 for root interpolation
		p1 = startPosture->root_pos;
		p2 = endPosture->root_pos;

		if (startKeyframe == 0)
		{
			//first key frame
			firstKF = true;
			p3 = nextPosture->root_pos;
		}
		else if (nextKeyFrame > inputLength)
		{
			//last key frame
			lastKF = true;
			p0 = prePosture->root_pos;
		}
		else
		{
			//inter frame
			p0 = prePosture->root_pos;
			p3 = nextPosture->root_pos;
			firstKF = false;
			lastKF = false;
		}
		ControlPointEuler(p0, p1, p2, p3, a1, b2, firstKF, lastKF);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a1, b2, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);

				if (startKeyframe == 0)
				{
					//first key frame
					firstKFBone = true;
					Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
				}
				else if (nextKeyFrame > inputLength) {
					//last key frame
					lastKFBone = true;
					Euler2Quaternion(prePosture->bone_rotation[bone].p, q0);
				}
				else
				{
					//inter frame
					Euler2Quaternion(prePosture->bone_rotation[bone].p, q0);
					Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
					firstKFBone = false;
					lastKFBone = false;
				}
				QuaternionControlPoint(q0, q1, q2, q3, an, bn, firstKFBone, lastKFBone);
				qInter = DeCasteljauQuaternion(t, q1, an, bn, q2);
				Quaternion2Euler(qInter, interpolatedPosture.bone_rotation[bone].p);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
	// students should implement this
	double R[9];
	Euler2Rotation(angles, R);
	q = Quaternion<double>::Matrix2Quaternion(R);
	q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3])
{
	// students should implement this
	double R[9];
	q.Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
	// students should implement this
	Quaternion<double> result;
	double cosTheta = qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz() + qStart.Gets() * qEnd_.Gets();
	double theta = acos(cosTheta);

	//always choose the shortest path
	if (cosTheta < 0) {
		theta = M_PI - acos(-cosTheta);
		qEnd_ = qEnd_ * (-1.0);
	}

	//if theta equals to 0 reutrn qStart
	if (sin(theta) == 0.0) {
		result = qStart;
		return result;
	}

	result = sin((1 - t) * theta) / sin(theta) * qStart + sin(t * theta) / sin(theta) * qEnd_;
	result.Normalize();
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
	// students should implement this
	Quaternion<double> result;
	result = Slerp(2.0, p, q);
	return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
	// students should implement this
	vector result;
	vector q0, q1, q2, r0, r1;
	q0 = p0 + (p1 - p0) * t;
	q1 = p1 + (p2 - p1) * t;
	q2 = p2 + (p3 - p2) * t;
	r0 = q0 + (q1 - q0) * t;
	r1 = q1 + (q2 - q1) * t;
	result = r0 + (r1 - r0) * t;
	return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
	// students should implement this
	Quaternion<double> result;
	Quaternion<double> q0, q1, q2, r0, r1;
	q0 = Slerp(t, p0, p1);
	q1 = Slerp(t, p1, p2);
	q2 = Slerp(t, p2, p3);
	r0 = Slerp(t, q0, q1);
	r1 = Slerp(t, q1, q2);
	result = Slerp(t, r0, r1);
	return result;
}