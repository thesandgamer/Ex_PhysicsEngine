#include "Intersections.h"


/// <summary>
/// Pour savoir si deux corps collisionnent
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
bool Intersections::Intersect(Body& a, Body& b, const float dt, Contact& contact)
{
	contact.a = &a;
	contact.b = &b;
	const Vec3 ab = b.position - a.position;

	contact.normal = ab;
	contact.normal.Normalize();

	if (a.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE && b.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE) //Si les deux sont des sphères
	{
		ShapeSphere* sphereA = static_cast<ShapeSphere*>(a.shape);
		ShapeSphere* sphereB = static_cast<ShapeSphere*>(b.shape);

		Vec3 posA = a.position;
		Vec3 posB = b.position;
		Vec3 valA = a.linearVelocity;
		Vec3 velB = b.linearVelocity;

		if (Intersections::SphereSphereDynamic(*sphereA, *sphereB,posA, posB, valA, velB, dt,contact.ptOnAWorldSpace, contact.ptOnBWorldSpace,contact.timeOfImpact))	//Si il y a une futur collision
		{
			// Step bodies forward to get local space collision points: Met à jour les collisions au temps du point d'impact pour mettre a jour les info du contact
			a.Update(contact.timeOfImpact);
			b.Update(contact.timeOfImpact);

			// Convert world space contacts to local space
			contact.ptOnALocalSpace = a.WorldSpaceToBodySpace(contact.ptOnAWorldSpace);
			contact.ptOnBLocalSpace = b.WorldSpaceToBodySpace(contact.ptOnBWorldSpace);

			Vec3 ab = a.position - b.position;
			contact.normal = ab;
			contact.normal.Normalize();

			// Unwind time step: Puis on remet les collisions à l'étape d'avant
			a.Update(-contact.timeOfImpact);
			b.Update(-contact.timeOfImpact);

			// Calculate separation distance
			float r = ab.GetMagnitude()	- (sphereA->radius + sphereB->radius);
			contact.separationDistance = r;

			return true;
		}
	}
	return false;

}

/// <summary>
/// Si un rayon touche une sphère
/// </summary>
/// <param name="rayStart"></param>
/// <param name="rayDir"></param>
/// <param name="sphereCenter"></param>
/// <param name="sphereRadius"></param>
/// <param name="t0"></param>
/// <param name="t1"></param>
/// <returns></returns>
bool Intersections::RaySphere(const Vec3& rayStart, const Vec3& rayDir, const Vec3& sphereCenter, const float sphereRadius, float& t0, float& t1)
{
	const Vec3& s = sphereCenter - rayStart;
	const float a = rayDir.Dot(rayDir);
	const float b = s.Dot(rayDir);
	const float c = s.Dot(s) - sphereRadius * sphereRadius;

	const float delta = b * b - a * c;
	const float inverseA = 1.0f / a;

	if (delta < 0) {
		// No solution
		return false;
	}
	const float deltaRoot = sqrtf(delta);
	t0 = (b - deltaRoot) * inverseA;
	t1 = (b + deltaRoot) * inverseA;

	return true;
}

bool Intersections::SphereSphereDynamic(const ShapeSphere& shapeA, const ShapeSphere& shapeB, const Vec3& posA, const Vec3& posB, 
											const Vec3& velA, const Vec3& velB, const float dt, Vec3& ptOnA, Vec3& ptOnB, float& timeOfImpact)
{
	const Vec3 relativeVelocity = velA - velB;
	const Vec3 startPtA = posA;
	const Vec3 endPtA = startPtA + relativeVelocity * dt;
	const Vec3 rayDir = endPtA - startPtA;

	float t0 = 0;//Temps présent toujours à zero
	float t1 = 0;//Temps futur

	if (rayDir.GetLengthSqr() < 0.001f * 0.001f)//Si le rayon est trop court
	{
		// Ray is too short, just check if already intersecting
		Vec3 ab = posB - posA;
		float radius = shapeA.radius + shapeB.radius + 0.001f;
		if (ab.GetLengthSqr() > radius * radius)	//Calcul normal de collision entre sphères
		{
			return false;
		}
	}
	else if (!RaySphere(startPtA, rayDir, posB, shapeA.radius + shapeB.radius, t0, t1))//Sinon si le rayon ne collide pas avec une sphère
	{
		return false;
	}

	// Change from [0, 1] to [0, dt];
	t0 *= dt;
	t1 *= dt;//On avance dans le futur

	// If the collision in only in the past, there will be no future collision for this frame
	if (t1 < 0) return false;

	// Get earliest positive time of impact
	timeOfImpact = t0 < 0.0f ? 0.0f : t0;//If t0 < 0 : timeOfImpact = 0 | else timeOfImpact = t0

	// If the earliest collision is too far in the future, then there's no collision this frame
	if (timeOfImpact > dt)//Si le temps futur est à la frame d'après 
	{
		return false;
	}

	// Get the points on the respective points of collision and return true

	//Calcul les positions au temps futur
	Vec3 newPosA = posA + velA * timeOfImpact;
	Vec3 newPosB = posB + velB * timeOfImpact;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	ptOnA = newPosA + ab * shapeA.radius;
	ptOnB = newPosB - ab * shapeB.radius;

	return true;

}