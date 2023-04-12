//
//  Scene.cpp
//
#include "Scene.h"
#include "../Shape.h"
#include "../Intersections.h"
#include "../Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;

	// Tests with one moving fast an oter fix 
	/*
	Body fast;
	fast.position = Vec3(-3, 0, 3);
	fast.orientation = Quat(0, 0, 0, 1);
	fast.shape = new ShapeSphere(1.0f);
	fast.inverseMass = 1.0f;
	fast.elasticity = 0.5f;
	fast.friction = 0.5f;
	fast.linearVelocity = Vec3(500, 0, 0);
	bodies.push_back(fast);

	Body immobile;
	immobile.position = Vec3(0, 0, 3);
	immobile.orientation = Quat(0, 0, 0, 1);
	immobile.shape = new ShapeSphere(1.0f);
	immobile.inverseMass = 1.0f;
	immobile.elasticity = 0.5f;
	immobile.friction = 0.5f;
	immobile.linearVelocity = Vec3(0, 0, 0);
	bodies.push_back(immobile);

	Body earth;
	earth.position = Vec3(0, 0, -1000);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(1000.0f);
	earth.inverseMass = 0.0f;
	earth.elasticity = 0.99f;
	earth.friction = 0.5f;
	bodies.push_back(earth);
	*/

	/*
	
	//Rajoutes plusiueurs sphères
	for (int i = 0; i < 6; ++i)
	{
		for (int j = 0; j < 6; ++j)
		{
			float radius = 0.5f;
			float x = (i - 1) * radius * 1.5f;
			float y = (j - 1) * radius * 1.5f;
			body.position = Vec3(x, y, 10);
			body.orientation = Quat(0, 0, 0, 1);
			body.shape = new ShapeSphere(radius);
			body.inverseMass = 1.0f;
			body.elasticity = 0.3f;
			body.friction = 0.999f;
			body.linearVelocity.Zero();
			bodies.push_back(body);
		}
	}

	//---Rajoute le sol
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			float radius = 80.0f;
			float x = (i - 1) * radius * 0.25f;
			float y = (j - 1) * radius * 0.25f;
			body.position = Vec3(x, y, -radius);
			body.orientation = Quat(0, 0, 0, 1);
			body.shape = new ShapeSphere(radius);
			body.inverseMass = 0.0f;
			body.elasticity = 0.99f;
			body.friction = 0.5f;
			bodies.push_back(body);
		}
	}

	*/

	
	
	body.position = Vec3( 0, 0, 10 );
	body.orientation = Quat( 0, 0, 0, 1 );
	body.shape = new ShapeSphere( 1.0f );
	body.inverseMass = 1.0f;
	body.elasticity = 0.5f;
	body.friction = 0.5f;
	bodies.push_back( body );
	
	Body earth;
	earth.position = Vec3(0, 0, -6000);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(6000.0f);
	earth.inverseMass = 0.0f;
	earth.elasticity = 1.0f;
	earth.friction = 0.5f;
	bodies.push_back(earth);
	

	// TODO: Add code
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	// TODO: Add code

	//Gravité 
	for (int i = 0; i < bodies.size(); ++i) 
	{
		Body& body = bodies[i];
		float mass = 1.0f / body.inverseMass;
		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;	//Créer impulse de gravité
		body.ApplyImpulseLinear(impulseGravity);	//Applique l'impulse
	}

	

	// Broadphase
	std::vector<CollisionPair> collisionPairs;
	BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt_sec);
	// Collision checks (Narrow phase)
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)alloca(sizeof(Contact) * maxContacts);
	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		const CollisionPair& pair = collisionPairs[i];
		Body& bodyA = bodies[pair.a];
		Body& bodyB = bodies[pair.b];
		if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
			continue;
		Contact contact;
		if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			++numContacts;
		}
	}

	// Sort times of impact
	if (numContacts > 1)
	{
		qsort(contacts, numContacts, sizeof(Contact), Contact::CompareContact);
	}

	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;
		// Skip body par with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
			continue;
		// Position update
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].Update(dt);
		}
		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}

	// Other physics behavirous, outside collisions.
	// Update the positions for the rest of this frame's time.
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i].Update(timeRemaining);
		}
	}

}


