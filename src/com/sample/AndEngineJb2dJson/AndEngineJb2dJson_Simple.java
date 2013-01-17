package com.sample.AndEngineJb2dJson;

/*
 Author: Chris Campbell - www.iforce2d.net

 This software is provided 'as-is', without any express or implied
 warranty.  In no event will the authors be held liable for any damages
 arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it
 freely, subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.Writer;
import java.io.InputStream;
import java.io.FileInputStream;
import java.io.BufferedReader;
import java.lang.reflect.Array;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Map;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;
import java.util.Vector;

import org.andengine.extension.physics.box2d.*;
import org.andengine.util.debug.Debug;
import org.andengine.util.math.MathUtils;

import android.hardware.SensorManager;

import com.badlogic.gdx.physics.box2d.*;
import com.badlogic.gdx.physics.box2d.BodyDef.BodyType;
import com.badlogic.gdx.physics.box2d.joints.DistanceJointDef;
import com.badlogic.gdx.physics.box2d.joints.FrictionJointDef;
import com.badlogic.gdx.physics.box2d.joints.GearJointDef;
import com.badlogic.gdx.physics.box2d.joints.MouseJoint;
import com.badlogic.gdx.physics.box2d.joints.MouseJointDef;
import com.badlogic.gdx.physics.box2d.joints.PrismaticJoint;
import com.badlogic.gdx.physics.box2d.joints.PrismaticJointDef;
import com.badlogic.gdx.physics.box2d.joints.PulleyJointDef;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJoint;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;
import com.badlogic.gdx.physics.box2d.joints.WeldJointDef;
import com.badlogic.gdx.math.Vector2;

import org.jbox2d.common.Settings;
import org.json.*;

//Partially Ported to AndEngine: Bart Hirst - www.linkedin.com/in/louisbhirst

public class AndEngineJb2dJson_Simple {

	protected boolean m_useHumanReadableFloats;

	protected int m_simulationPositionIterations;
	protected int m_simulationVelocityIterations;
	protected float m_simulationFPS;

	protected Map<Integer, Body> m_indexToBodyMap;
	protected Map<Body, Integer> m_bodyToIndexMap;
	protected Map<Joint, Integer> m_jointToIndexMap;
	protected Vector<Body> m_bodies;
	protected Vector<Joint> m_joints;
	protected Vector<Jb2dJsonImage> m_images;

	protected Map<Body, String> m_bodyToNameMap;
	protected Map<Fixture, String> m_fixtureToNameMap;
	protected Map<Joint, String> m_jointToNameMap;
	protected Map<Jb2dJsonImage, String> m_imageToNameMap;

	public static final float JSON_SCALE = 1.0f;
		
	private static final int VELOCITY_ITERATIONS = 8;	//default
	private static final int POSITION_ITERATIONS = 8;	//default
	
	public AndEngineJb2dJson_Simple() {
		this(true);
	}

	public AndEngineJb2dJson_Simple(boolean useHumanReadableFloats) {

		if (!useHumanReadableFloats) {
			// The floatToHex function is not giving the same results
			// as the original C++ version... not critical so worry about it
			// later.
			System.out
					.println("Non human readable floats are not implemented yet");
			useHumanReadableFloats = true;
		}

		m_useHumanReadableFloats = useHumanReadableFloats;
		m_simulationPositionIterations = 3;
		m_simulationVelocityIterations = 8;
		m_simulationFPS = 60;

		m_indexToBodyMap = new HashMap<Integer, Body>();
		m_bodyToIndexMap = new HashMap<Body, Integer>();
		m_jointToIndexMap = new HashMap<Joint, Integer>();
		m_bodies = new Vector<Body>();
		m_joints = new Vector<Joint>();
		m_images = new Vector<Jb2dJsonImage>();

		m_bodyToNameMap = new HashMap<Body, String>();
		m_fixtureToNameMap = new HashMap<Fixture, String>();
		m_jointToNameMap = new HashMap<Joint, String>();
		m_imageToNameMap = new HashMap<Jb2dJsonImage, String>();
	}

	public void setBodyName(Body body, String name) {
		m_bodyToNameMap.put(body, name);
	}
	
	public void setFixtureName(Fixture fixture, String name) {
		m_fixtureToNameMap.put(fixture, name);
	}

	public void setJointName(Joint joint, String name) {
		m_jointToNameMap.put(joint, name);
	}

	public void setImageName(Jb2dJsonImage image, String name) {
		m_imageToNameMap.put(image, name);
	}

	public PhysicsWorld readFromString(String str, StringBuilder errorMsg) {
		try {
			JSONObject worldValue = new JSONObject(str);
			return j2b2World(worldValue);
		} catch (JSONException e) {
			errorMsg.append("Failed to parse JSON");
			e.printStackTrace();
			return null;
		}
	}

	public PhysicsWorld j2b2World(JSONObject worldValue) throws JSONException {
					
		PhysicsWorld world = new PhysicsWorld(jsonToVec("gravity", worldValue),  worldValue.getBoolean("allowSleep"), VELOCITY_ITERATIONS, POSITION_ITERATIONS);			
		
		world.setAutoClearForces(worldValue.getBoolean("autoClearForces"));
		world.setWarmStarting(worldValue.getBoolean("warmStarting"));
		world.setContinuousPhysics(worldValue.getBoolean("continuousPhysics"));
		
		int i = 0;
		JSONArray bodyValues = worldValue.optJSONArray("body");
		if (null != bodyValues) {
			int numBodyValues = bodyValues.length();
			for (i = 0; i < numBodyValues; i++) {
				Body body = j2b2Body(world, bodyValues.getJSONObject(i));
				m_bodies.add(body);
				m_indexToBodyMap.put(i, body);
			}
		}

		// need two passes for joints because gear joints reference other joints
		JSONArray jointValues = worldValue.optJSONArray("joint");
		if (null != jointValues) {
			int numJointValues = jointValues.length();
			for (i = 0; i < numJointValues; i++) {
				JSONObject jointValue = jointValues.getJSONObject(i);
				if ( ! jointValue.optString("type", "").equals("gear") ) {
					Joint joint = j2b2Joint(world, jointValue);
					m_joints.add(joint);
				}
			}
			for (i = 0; i < numJointValues; i++) {
				JSONObject jointValue = jointValues.getJSONObject(i);
				if ( jointValue.optString("type", "").equals("gear") ) {
					Joint joint = j2b2Joint(world, jointValue);
					m_joints.add(joint);
				}
			}
		}
		
		//todo images
		/*
		i = 0;
		JSONArray imageValues = worldValue.optJSONArray("image");
		if (null != imageValues) {
			int numImageValues = imageValues.length();
			for (i = 0; i < numImageValues; i++) {
				Jb2dJsonImage image = j2b2dJsonImage(imageValues.getJSONObject(i));
				m_images.add(image);
			}
		}
		*/

		return world;
		
	}
	
	Joint j2b2Joint(PhysicsWorld world, JSONObject jointValue) throws JSONException
    {
        Joint joint = null;

        int bodyIndexA = jointValue.getInt("bodyA");
        int bodyIndexB = jointValue.getInt("bodyB");
        if ( bodyIndexA >= m_bodies.size() || bodyIndexB >= m_bodies.size() )
            return null;

        //keep these in scope after the if/else below
        RevoluteJointDef revoluteDef;
        PrismaticJointDef prismaticDef;
        DistanceJointDef distanceDef;
        PulleyJointDef pulleyDef;
        MouseJointDef mouseDef;
        GearJointDef gearDef;
        //WheelJointDef wheelDef;
        WeldJointDef weldDef;
        FrictionJointDef frictionDef;
        //RopeJointDef ropeDef;

        //will be used to select one of the above to work with
        JointDef jointDef = null;

        Vector2 mouseJointTarget = new Vector2(0,0);
        String type = jointValue.optString("type","");
        if ( type.equals("revolute") )
        {
            jointDef = revoluteDef = new RevoluteJointDef();
            revoluteDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
            revoluteDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
            revoluteDef.referenceAngle = jsonToFloat("refAngle", jointValue);
            revoluteDef.enableLimit = jointValue.optBoolean("enableLimit",false);
            revoluteDef.lowerAngle = jsonToFloat("lowerLimit", jointValue);
            revoluteDef.upperAngle = jsonToFloat("upperLimit", jointValue);
            revoluteDef.enableMotor = jointValue.optBoolean("enableMotor",false);
            revoluteDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
            revoluteDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
        }
        else if ( type.equals("prismatic") )
        {
            jointDef = prismaticDef = new PrismaticJointDef();
            prismaticDef.localAnchorA.set( jsonToVec("anchorA", jointValue) );
            prismaticDef.localAnchorB.set( jsonToVec("anchorB", jointValue) );
            if ( jointValue.has("localAxisA") )
                prismaticDef.localAxis1.set( jsonToVec("localAxisA", jointValue) );
            else
                prismaticDef.localAxis1.set( jsonToVec("localAxis1", jointValue) );
            prismaticDef.referenceAngle = jsonToFloat("refAngle", jointValue);
            prismaticDef.enableLimit = jointValue.optBoolean("enableLimit");
            prismaticDef.lowerTranslation = jsonToFloat("lowerLimit", jointValue);
            prismaticDef.upperTranslation = jsonToFloat("upperLimit", jointValue);
            prismaticDef.enableMotor = jointValue.optBoolean("enableMotor");
            prismaticDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
            prismaticDef.maxMotorForce = jsonToFloat("maxMotorForce", jointValue);
        }
        else if ( type.equals("distance") )
        {
            jointDef = distanceDef = new DistanceJointDef();
            distanceDef.localAnchorA.set( jsonToVec("anchorA", jointValue) );
            distanceDef.localAnchorB.set( jsonToVec("anchorB", jointValue) );
            distanceDef.length = jsonToFloat("length", jointValue);
            distanceDef.frequencyHz = jsonToFloat("frequency", jointValue);
            distanceDef.dampingRatio = jsonToFloat("dampingRatio", jointValue);
        }
        else if ( type.equals("pulley") )
        {
            jointDef = pulleyDef = new PulleyJointDef();
            pulleyDef.groundAnchorA.set( jsonToVec("groundAnchorA", jointValue) );
            pulleyDef.groundAnchorB.set( jsonToVec("groundAnchorB", jointValue) );
            pulleyDef.localAnchorA.set( jsonToVec("anchorA", jointValue) );
            pulleyDef.localAnchorB.set( jsonToVec("anchorB", jointValue) );
            pulleyDef.lengthA = jsonToFloat("lengthA", jointValue);
            pulleyDef.lengthB = jsonToFloat("lengthB", jointValue);
            pulleyDef.ratio = jsonToFloat("ratio", jointValue);
        }
        else if ( type.equals("mouse") )
        {
            jointDef = mouseDef = new MouseJointDef();
            mouseJointTarget = jsonToVec("target", jointValue);
            mouseDef.target.set( jsonToVec("anchorB", jointValue) );//alter after creating joint
            mouseDef.maxForce = jsonToFloat("maxForce", jointValue);
            mouseDef.frequencyHz = jsonToFloat("frequency", jointValue);
            mouseDef.dampingRatio = jsonToFloat("dampingRatio", jointValue);
        }
		// Gear joints are apparently not implemented in JBox2D yet, but
		// when they are, commenting out the following section should work.
        /*
        else if ( type.equals("gear") )
        {
            jointDef = gearDef = new GearJointDef();
            int jointIndex1 = jointValue.getInt("joint1");
            int jointIndex2 = jointValue.getInt("joint2");
            gearDef.joint1 = m_joints.get(jointIndex1);
            gearDef.joint2 = m_joints.get(jointIndex2);
            gearDef.ratio = jsonToFloat("ratio", jointValue);
        }
        */
		// Wheel joints are apparently not implemented in JBox2D yet, but
		// when they are, commenting out the following section should work.
        /*
        else if ( type.equals("wheel") )
        {
            jointDef = wheelDef = new WheelJointDef();
            wheelDef.localAnchorA.set( jsonToVec("anchorA", jointValue) );
            wheelDef.localAnchorB.set( jsonToVec("anchorB", jointValue) );
            wheelDef.localAxisA.set( jsonToVec("localAxisA", jointValue) );
            wheelDef.enableMotor = jointValue.optBoolean("enableMotor",false);
            wheelDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
            wheelDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
            wheelDef.frequencyHz = jsonToFloat("springFrequency", jointValue);
            wheelDef.dampingRatio = jsonToFloat("springDampingRatio", jointValue);
        }
        */
        // For now, we will make do with a revolute joint.
        else if ( type.equals("wheel") )
        {
            jointDef = revoluteDef = new RevoluteJointDef();
            revoluteDef.localAnchorA.set(jsonToVec("anchorA", jointValue));
            revoluteDef.localAnchorB.set(jsonToVec("anchorB", jointValue));
            revoluteDef.enableMotor = jointValue.optBoolean("enableMotor",false);
            revoluteDef.motorSpeed = jsonToFloat("motorSpeed", jointValue);
            revoluteDef.maxMotorTorque = jsonToFloat("maxMotorTorque", jointValue);
        }
        else if ( type.equals("weld") )
        {
            jointDef = weldDef = new WeldJointDef();
            weldDef.localAnchorA.set( jsonToVec("anchorA", jointValue) );
            weldDef.localAnchorB.set( jsonToVec("anchorB", jointValue) );
            weldDef.referenceAngle = 0;
        }
        else if ( type.equals("friction") )
        {
            jointDef = frictionDef = new FrictionJointDef();
            frictionDef.localAnchorA.set( jsonToVec("anchorA", jointValue) );
            frictionDef.localAnchorB.set( jsonToVec("anchorB", jointValue) );
            frictionDef.maxForce = jsonToFloat("maxForce", jointValue);
            frictionDef.maxTorque = jsonToFloat("maxTorque", jointValue);
        }
		// Rope joints are apparently not implemented in JBox2D yet, but
		// when they are, commenting out the following section should work.
        /*
        else if ( type.equals("rope") )
        {
            jointDef = ropeDef = new RopeJointDef();
            ropeDef.localAnchorA.set( jsonToVec("anchorA", jointValue) );
            ropeDef.localAnchorB.set( jsonToVec("anchorB", jointValue) );
            ropeDef.maxLength = jsonToFloat("maxLength", jointValue);
        }
        */

        if ( null != jointDef ) {
            //set features common to all joints
            jointDef.bodyA = m_bodies.get(bodyIndexA);
            jointDef.bodyB = m_bodies.get(bodyIndexB);
            jointDef.collideConnected = jointValue.optBoolean("collideConnected",false);

            joint = world.createJoint(jointDef);

            if ( type.equals("mouse") )
                ((MouseJoint)joint).setTarget(mouseJointTarget);

            String jointName = jointValue.optString("name","");
            if ( ! jointName.equals("") ) {
                setJointName(joint, jointName);
            }
        }

        return joint;
    }

	public Body j2b2Body(PhysicsWorld physicsWorld, JSONObject bodyValue) throws JSONException {
		
		BodyDef bodyDef = new BodyDef();
		switch (bodyValue.getInt("type")) {
		case 0:
			bodyDef.type = BodyType.StaticBody;
			break;
		case 1:
			bodyDef.type = BodyType.KinematicBody;
			break;
		case 2:
			bodyDef.type = BodyType.DynamicBody;
			break;
		}
		
		bodyDef.position.set(jsonToVec("position", bodyValue));
		bodyDef.angle = jsonToFloat("angle", bodyValue);
		bodyDef.linearVelocity.set(jsonToVec("linearVelocity", bodyValue));
		bodyDef.angularVelocity = jsonToFloat("angularVelocity", bodyValue);
		bodyDef.linearDamping = jsonToFloat("linearDamping", bodyValue, -1, 0);
		bodyDef.angularDamping = jsonToFloat("angularDamping", bodyValue, -1, 0);
		//removing - Experimental: scales the inertia tensor.
		//bodyDef.gravityScale = jsonToFloat("gravityScale", bodyValue, -1, 1);

		bodyDef.allowSleep = bodyValue.optBoolean("allowSleep", true);
		bodyDef.awake = bodyValue.optBoolean("awake", false);
		bodyDef.fixedRotation = bodyValue.optBoolean("fixedRotation");
		bodyDef.bullet = bodyValue.optBoolean("bullet", false);
		bodyDef.active = bodyValue.optBoolean("active", true);
				
		Body body = physicsWorld.createBody(bodyDef);

		String bodyName = bodyValue.optString("name", "");
		if ("" != bodyName)
			setBodyName(body, bodyName);

		int i = 0;
		JSONArray fixtureValues = bodyValue.optJSONArray("fixture");
		if (null != fixtureValues) {
			int numFixtureValues = fixtureValues.length();
			for (i = 0; i < numFixtureValues; i++) {
				Fixture fixture = j2b2Fixture(body, fixtureValues.getJSONObject(i));
			}
		}

		// may be necessary if user has overridden mass characteristics
		MassData massData = new MassData();
		massData.mass = jsonToFloat("massData-mass", bodyValue);
		massData.center.set(jsonToVec("massData-center", bodyValue));
		massData.I = jsonToFloat("massData-I", bodyValue);
		body.setMassData(massData);

		return body;
	}
	
	float jsonToFloat(String name, JSONObject value) {
		return jsonToFloat(name, value, -1, 0);
	}
	
	float jsonToFloat(String name, JSONObject value, int index) {
		return jsonToFloat(name, value, index, 0);
	}

	float jsonToFloat(String name, JSONObject value, int index, float defaultValue) {
		if (!value.has(name))
			return defaultValue;

		if (index > -1) {
			JSONArray array = null;
			try {
				array = value.getJSONArray(name);
			} catch (JSONException e) {
			}
			if (null == array)
				return defaultValue;
			Object obj = array.opt(index);
			if (null == obj)
				return defaultValue;
			// else if ( value[name].isString() )
			// return hexToFloat( value[name].asString() );
			else
				return ((Number) obj).floatValue() * JSON_SCALE;
		} else {
			Object obj = value.opt(name);
			if (null == obj)
				return defaultValue;
			// else if ( value[name].isString() )
			// return hexToFloat( value[name].asString() );
			else
				return ((Number) obj).floatValue() * JSON_SCALE;
		}
	}
	
	Fixture j2b2Fixture(Body body, JSONObject fixtureValue) throws JSONException {

		if (null == fixtureValue)
			return null;

		Debug.d("Adding fixture : " + fixtureValue.optString("name", ""));
		
		FixtureDef fixtureDef = new FixtureDef();
		fixtureDef.restitution = jsonToFloat("restitution", fixtureValue);
		fixtureDef.friction = jsonToFloat("friction", fixtureValue);
		fixtureDef.density = jsonToFloat("density", fixtureValue);
		fixtureDef.isSensor = fixtureValue.optBoolean("sensor", false);

		//casted to short - hopefully not a problem
		fixtureDef.filter.categoryBits = (short) fixtureValue.optInt("filter-categoryBits", 0x0001);
		fixtureDef.filter.maskBits = (short) fixtureValue.optInt("filter-maskBits", 0xffff);
		fixtureDef.filter.groupIndex = (short) fixtureValue.optInt("filter-groupIndex", 0);

		Fixture fixture = null;
		if (null != fixtureValue.optJSONObject("circle")) {
			
			JSONObject circleValue = fixtureValue.getJSONObject("circle");
			CircleShape circleShape = new CircleShape();
						
			final float radius = jsonToFloat("radius", circleValue);
			Debug.d("radius: " + radius);
			circleShape.setRadius(jsonToFloat("radius", circleValue));
			circleShape.setPosition(jsonToVec("center", circleValue));
					
			fixtureDef.shape = circleShape;
			fixture = body.createFixture(fixtureDef);
			circleShape.dispose();			
				
		//edges and chains don't appear to be supported in com.badlogic.gdx.physics.box2d
		/*
		} else if (null != fixtureValue.optJSONObject("edge")) {
			JSONObject edgeValue = fixtureValue.getJSONObject("edge");
			EdgeShape edgeShape = new EdgeShape();
			edgeShape.m_vertex1.set(jsonToVec("vertex1", edgeValue));
			edgeShape.m_vertex2.set(jsonToVec("vertex2", edgeValue));
			edgeShape.m_hasVertex0 = edgeValue.optBoolean("hasVertex0", false);
			edgeShape.m_hasVertex3 = edgeValue.optBoolean("hasVertex3", false);
			if (edgeShape.m_hasVertex0)
				edgeShape.m_vertex0.set(jsonToVec("vertex0", edgeValue));
			if (edgeShape.m_hasVertex3)
				edgeShape.m_vertex3.set(jsonToVec("vertex3", edgeValue));
			fixtureDef.shape = edgeShape;
			fixture = body.createFixture(fixtureDef);
		} else if (null != fixtureValue.optJSONObject("loop")) {// support old
																// format (r197)
			JSONObject chainValue = fixtureValue.getJSONObject("loop");
			ChainShape chainShape = new ChainShape();
			int numVertices = chainValue.getJSONArray("x").length();
			Vector2 vertices[] = new Vector2[numVertices];
			for (int i = 0; i < numVertices; i++)
				vertices[i].set(jsonToVec("vertices", chainValue, i));
			chainShape.createLoop(vertices, numVertices);
			fixtureDef.shape = chainShape;
			fixture = body.createFixture(fixtureDef);
		} else if (null != fixtureValue.optJSONObject("chain")) {
			JSONObject chainValue = fixtureValue.getJSONObject("chain");
			ChainShape chainShape = new ChainShape();
			int numVertices = chainValue.getJSONObject("vertices")
					.getJSONArray("x").length();
			Vector2 vertices[] = new Vector2[numVertices];
			for (int i = 0; i < numVertices; i++)
				vertices[i] = jsonToVec("vertices", chainValue, i);
			chainShape.createChain(vertices, numVertices);
			chainShape.m_hasPrevVertex = chainValue.optBoolean("hasPrevVertex",
					false);
			chainShape.m_hasNextVertex = chainValue.optBoolean("hasNextVertex",
					false);
			if (chainShape.m_hasPrevVertex)
				chainShape.m_prevVertex
						.set(jsonToVec("prevVertex", chainValue));
			if (chainShape.m_hasNextVertex)
				chainShape.m_nextVertex
						.set(jsonToVec("nextVertex", chainValue));
			fixtureDef.shape = chainShape;
			fixture = body.createFixture(fixtureDef);
		*/
		} else if (null != fixtureValue.optJSONObject("polygon")) {
								
			JSONObject polygonValue = fixtureValue.getJSONObject("polygon");
			
			int numVertices = polygonValue.getJSONObject("vertices").getJSONArray("x").length();
			Vector2 vertices[] = new Vector2[numVertices];
			
			if (numVertices > Settings.maxPolygonVertices) {
				System.out
						.println("Ignoring polygon fixture with too many vertices.");
			} else if (numVertices < 2) {
				System.out
						.println("Ignoring polygon fixture less than two vertices.");					
			} else if (numVertices == 2) {
				//Debug.d("loadWorld error: " + errorMsg);
				System.out
						.println("Creating edge shape instead of polygon with two vertices.");			
				
				//edges unsupported	
				//TODO: fix this
				
				//EdgeShape edgeShape = new EdgeShape();
				//edgeShape.m_vertex1.set(jsonToVec("vertices", polygonValue, 0));
				//edgeShape.m_vertex2.set(jsonToVec("vertices", polygonValue, 1));
				//fixtureDef.shape = edgeShape;
				//fixture = body.createFixture(fixtureDef);
			} else {

				PolygonShape polygonShape = new PolygonShape();

				for (int i = 0; i < numVertices; i++)		{								
					vertices[i] = jsonToVec("vertices", polygonValue, i);
				}
			
				polygonShape.set(vertices);
				fixtureDef.shape = polygonShape;
				fixture = body.createFixture(fixtureDef);
			}
		}

		String fixtureName = fixtureValue.optString("name", "");
		if (fixtureName != "") {
			setFixtureName(fixture, fixtureName);
		}

		return fixture;
	}

	Vector2 jsonToVec(String name, JSONObject value) throws JSONException {
		return jsonToVec(name, value, -1, new Vector2(0, 0));
	}

	Vector2 jsonToVec(String name, JSONObject value, int index)
			throws JSONException {
		return jsonToVec(name, value, index, new Vector2(0, 0));
	}

	Vector2 jsonToVec(String name, JSONObject value, int index, Vector2 defaultValue)
			throws JSONException {
		Vector2 vec = defaultValue;

		if (!value.has(name))
			return defaultValue;

		if (index > -1) {
			JSONObject vecValue = value.getJSONObject(name);
			JSONArray arrayX = vecValue.getJSONArray("x");
			JSONArray arrayY = vecValue.getJSONArray("y");
			// if ( arrayX[index].isString() )
			// vec.x = hexToFloat(value[name]["x"][index].asString());
			// else
			vec.x = (float) arrayX.getDouble(index) * JSON_SCALE;

			// if ( arrayX[index].isString() )
			// vec.y = hexToFloat(value[name]["y"][index].asString());
			// else
			vec.y = (float) arrayY.getDouble(index) * JSON_SCALE;
		} else {
			JSONObject vecValue = value.optJSONObject(name);
			if (null == vecValue)
				return defaultValue;
			else if (!vecValue.has("x")) // should be zero vector
				vec.set(0, 0);
			else {
				vec.x = jsonToFloat("x", vecValue);
				vec.y = jsonToFloat("y", vecValue);
			}
		}

		return vec;
	}
	
	public Body[] getBodiesByName(String name) 
	{
		Set<Body> keys = new HashSet<Body>();
	     for (Entry<Body, String> entry : m_bodyToNameMap.entrySet()) {
	         if (name.equals(entry.getValue())) {
	             keys.add(entry.getKey());
	         }
	     }
	     return keys.toArray(new Body[0]);
	}
	
	public Fixture[] getFixturesByName(String name) 
	{
		Set<Fixture> keys = new HashSet<Fixture>();
	     for (Entry<Fixture, String> entry : m_fixtureToNameMap.entrySet()) {
	         if (name.equals(entry.getValue())) {
	             keys.add(entry.getKey());
	         }
	     }
	     return keys.toArray(new Fixture[0]);
	}
	
	public Joint[] getJointsByName(String name) 
	{
		Set<Joint> keys = new HashSet<Joint>();
	     for (Entry<Joint, String> entry : m_jointToNameMap.entrySet()) {
	         if (name.equals(entry.getValue())) {
	             keys.add(entry.getKey());
	         }
	     }
	     return keys.toArray(new Joint[0]);
	}
	
	public Jb2dJsonImage[] getImagesByName(String name) 
	{
		Set<Jb2dJsonImage> keys = new HashSet<Jb2dJsonImage>();
	     for (Entry<Jb2dJsonImage, String> entry : m_imageToNameMap.entrySet()) {
	         if (name.equals(entry.getValue())) {
	             keys.add(entry.getKey());
	         }
	     }
	     return keys.toArray(new Jb2dJsonImage[0]);
	}
	
	public Jb2dJsonImage[] getAllImages()
	{
	    return (Jb2dJsonImage[]) m_images.toArray();
	}

	public Body getBodyByName(String name)
	{
		for (Entry<Body, String> entry : m_bodyToNameMap.entrySet()) {
	        if (name.equals(entry.getValue())) {
	            return entry.getKey();
	        }
	    }
	    return null;
	}
	
	public Fixture getFixtureByName(String name)
	{
		for (Entry<Fixture, String> entry : m_fixtureToNameMap.entrySet()) {
	        if (name.equals(entry.getValue())) {
	            return entry.getKey();
	        }
	    }
	    return null;
	}
	
	public Joint getJointByName(String name)
	{
		for (Entry<Joint, String> entry : m_jointToNameMap.entrySet()) {
	        if (name.equals(entry.getValue())) {
	            return entry.getKey();
	        }
	    }
	    return null;
	}
	
	public Jb2dJsonImage getImageByName(String name)
	{
		for (Entry<Jb2dJsonImage, String> entry : m_imageToNameMap.entrySet()) {
	        if (name.equals(entry.getValue())) {
	            return entry.getKey();
	        }
	    }
	    return null;
	}
		
}
