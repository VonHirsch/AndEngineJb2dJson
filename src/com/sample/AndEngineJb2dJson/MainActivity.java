package com.sample.AndEngineJb2dJson;

/*
Author: Bart Hirst - www.linkedin.com/in/louisbhirst

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

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

import org.andengine.engine.camera.ZoomCamera;
import org.andengine.engine.camera.hud.controls.AnalogOnScreenControl;
import org.andengine.engine.camera.hud.controls.BaseOnScreenControl;
import org.andengine.engine.camera.hud.controls.AnalogOnScreenControl.IAnalogOnScreenControlListener;
import org.andengine.engine.handler.IUpdateHandler;
import org.andengine.engine.options.EngineOptions;
import org.andengine.engine.options.ScreenOrientation;
import org.andengine.engine.options.resolutionpolicy.RatioResolutionPolicy;
import org.andengine.entity.scene.IOnSceneTouchListener;
import org.andengine.entity.scene.Scene;
import org.andengine.entity.util.FPSLogger;
import org.andengine.extension.debugdraw.DebugRenderer;
import org.andengine.extension.physics.box2d.PhysicsWorld;
import org.andengine.extension.physics.box2d.util.constants.PhysicsConstants;
import org.andengine.input.touch.TouchEvent;
import org.andengine.input.touch.controller.MultiTouch;
import org.andengine.input.touch.detector.PinchZoomDetector;
import org.andengine.input.touch.detector.PinchZoomDetector.IPinchZoomDetectorListener;
import org.andengine.input.touch.detector.ScrollDetector;
import org.andengine.input.touch.detector.ScrollDetector.IScrollDetectorListener;
import org.andengine.input.touch.detector.SurfaceScrollDetector;
import org.andengine.opengl.texture.TextureOptions;
import org.andengine.opengl.texture.atlas.bitmap.BitmapTextureAtlas;
import org.andengine.opengl.texture.atlas.bitmap.BitmapTextureAtlasTextureRegionFactory;
import org.andengine.opengl.texture.region.ITextureRegion;
import org.andengine.ui.activity.SimpleBaseGameActivity;
import org.andengine.util.debug.Debug;

import com.badlogic.gdx.physics.box2d.Body;
import com.sample.AndEngineJb2dJson.R;

import android.opengl.GLES20;
import android.widget.Toast;

public class MainActivity extends SimpleBaseGameActivity implements IOnSceneTouchListener, IScrollDetectorListener, IPinchZoomDetectorListener {
	
	// ===========================================================
	// Constants
	// ===========================================================

	private static final int CAMERA_WIDTH = 720;
	private static final int CAMERA_HEIGHT = 480;

	// ===========================================================
	// Fields
	// ===========================================================

	private ZoomCamera mZoomCamera;

	private Scene mScene;	
	private DebugRenderer mDebugRenderer;
	private Body wheelBody = null;
	private float driveState = 0;

	private SurfaceScrollDetector mScrollDetector;
	private PinchZoomDetector mPinchZoomDetector;
	private float mPinchZoomStartedCameraZoomFactor;

	protected PhysicsWorld mPhysicsWorld;
	
	private BitmapTextureAtlas mOnScreenControlTexture;
	private ITextureRegion mOnScreenControlBaseTextureRegion;
	private ITextureRegion mOnScreenControlKnobTextureRegion;
	
	// ===========================================================
	// Constructors
	// ===========================================================

	// ===========================================================
	// Getter & Setter
	// ===========================================================

	// ===========================================================
	// Methods for/from SuperClass/Interfaces
	// ===========================================================

	@Override
	public EngineOptions onCreateEngineOptions() {
		this.mZoomCamera = new ZoomCamera(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);

		final EngineOptions engineOptions = new EngineOptions(true, ScreenOrientation.LANDSCAPE_FIXED, new RatioResolutionPolicy(CAMERA_WIDTH, CAMERA_HEIGHT), this.mZoomCamera);

		if(MultiTouch.isSupported(this)) {
			if(MultiTouch.isSupportedDistinct(this)) {
				//Toast.makeText(this, "MultiTouch detected --> Both controls will work properly!", Toast.LENGTH_SHORT).show();
			} else {
				Toast.makeText(this, "MultiTouch detected, but your device has problems distinguishing between fingers.\n\nControls are placed at different vertical locations.", Toast.LENGTH_LONG).show();
			}
		} else {
			Toast.makeText(this, "Sorry your device does NOT support MultiTouch!\n\n(Falling back to SingleTouch.)\n\nControls are placed at different vertical locations.", Toast.LENGTH_LONG).show();
		}

		return engineOptions;
	}

	@Override
	public void onCreateResources() {
		
		BitmapTextureAtlasTextureRegionFactory.setAssetBasePath("gfx/");

		this.mOnScreenControlTexture = new BitmapTextureAtlas(this.getTextureManager(), 256, 128, TextureOptions.BILINEAR);
		this.mOnScreenControlBaseTextureRegion = BitmapTextureAtlasTextureRegionFactory.createFromAsset(this.mOnScreenControlTexture, this, "onscreen_control_base.png", 0, 0);
		this.mOnScreenControlKnobTextureRegion = BitmapTextureAtlasTextureRegionFactory.createFromAsset(this.mOnScreenControlTexture, this, "onscreen_control_knob.png", 128, 0);
		this.mOnScreenControlTexture.load();

	}

	@Override
	public Scene onCreateScene() {
		
		this.mEngine.registerUpdateHandler(new FPSLogger());

		this.mScene = new Scene();
		
		//json files from RUBE are reversed in the Y direction
		mScene.setScale(1, -1);
		
		initPhysics();
		initControlAnalog();
			
		//this runs the wheel
		this.mScene.registerUpdateHandler(new IUpdateHandler() {
			
			@Override
			public void onUpdate(float pSecondsElapsed) {
				
				if (wheelBody != null) {					
					float maxSpeed = 75;	
					wheelBody.setAngularVelocity(maxSpeed*driveState);					
				}
				
				//uncomment this to have the camera track with the car
				//mZoomCamera.setCenter(wheelBody.getPosition().x*PhysicsConstants.PIXEL_TO_METER_RATIO_DEFAULT, wheelBody.getPosition().y*PhysicsConstants.PIXEL_TO_METER_RATIO_DEFAULT);
                
			}

			@Override
			public void reset() {
			}
		});

		this.mScrollDetector = new SurfaceScrollDetector(this);
		this.mPinchZoomDetector = new PinchZoomDetector(this);

		this.mScene.setOnSceneTouchListener(this);
		return this.mScene;
	}
	
	@Override
	public void onScrollStarted(final ScrollDetector pScollDetector, final int pPointerID, final float pDistanceX, final float pDistanceY) {
		final float zoomFactor = this.mZoomCamera.getZoomFactor();
		this.mZoomCamera.offsetCenter(-pDistanceX / zoomFactor, -pDistanceY / zoomFactor);
	}

	@Override
	public void onScroll(final ScrollDetector pScollDetector, final int pPointerID, final float pDistanceX, final float pDistanceY) {
		final float zoomFactor = this.mZoomCamera.getZoomFactor();
		this.mZoomCamera.offsetCenter(-pDistanceX / zoomFactor, -pDistanceY / zoomFactor);
	}
	
	@Override
	public void onScrollFinished(final ScrollDetector pScollDetector, final int pPointerID, final float pDistanceX, final float pDistanceY) {
		final float zoomFactor = this.mZoomCamera.getZoomFactor();
		this.mZoomCamera.offsetCenter(-pDistanceX / zoomFactor, -pDistanceY / zoomFactor);
	}

	@Override
	public void onPinchZoomStarted(final PinchZoomDetector pPinchZoomDetector, final TouchEvent pTouchEvent) {
		this.mPinchZoomStartedCameraZoomFactor = this.mZoomCamera.getZoomFactor();
	}

	@Override
	public void onPinchZoom(final PinchZoomDetector pPinchZoomDetector, final TouchEvent pTouchEvent, final float pZoomFactor) {
		this.mZoomCamera.setZoomFactor(this.mPinchZoomStartedCameraZoomFactor * pZoomFactor);
	}

	@Override
	public void onPinchZoomFinished(final PinchZoomDetector pPinchZoomDetector, final TouchEvent pTouchEvent, final float pZoomFactor) {
		this.mZoomCamera.setZoomFactor(this.mPinchZoomStartedCameraZoomFactor * pZoomFactor);
	}

	@Override
	public boolean onSceneTouchEvent(final Scene pScene, final TouchEvent pSceneTouchEvent) {		
		
		this.mPinchZoomDetector.onTouchEvent(pSceneTouchEvent);

		if(this.mPinchZoomDetector.isZooming()) {
			this.mScrollDetector.setEnabled(false);
		} else {
			if(pSceneTouchEvent.isActionDown()) {
				this.mScrollDetector.setEnabled(true);
			}
			this.mScrollDetector.onTouchEvent(pSceneTouchEvent);
		}
			
		return true;
		
	}

	// ===========================================================
	// Methods
	// ===========================================================
	
	private void initPhysics() {
		
		// if you want to play around with some of the other json's have a look at
		// Chris's Jb2dJson-JBox2D-src project under jbox2d\jbox2d-testbed\src\main\java\org\jbox2d\testbed\tests\LoadRUBE...
		loadJson(R.raw.car);
		
		mDebugRenderer = new DebugRenderer(mPhysicsWorld, getVertexBufferObjectManager());
		this.mScene.attachChild(mDebugRenderer);
		this.mScene.registerUpdateHandler(this.mPhysicsWorld);
		
	}
	
	private void loadJson(int resId) {
		
		AndEngineJb2dJson_Simple json = new AndEngineJb2dJson_Simple();
		StringBuilder errorMsg = new StringBuilder();
		this.mPhysicsWorld = json.readFromString(readRawTextFile(resId), errorMsg);
		wheelBody = json.getBodyByName("carwheel");	
		if (wheelBody != null) this.mZoomCamera.setCenter(wheelBody.getPosition().x*PhysicsConstants.PIXEL_TO_METER_RATIO_DEFAULT, wheelBody.getPosition().y*PhysicsConstants.PIXEL_TO_METER_RATIO_DEFAULT);
		
	}
		
	private void initControlAnalog() {

		final AnalogOnScreenControl analogOnScreenControl = new AnalogOnScreenControl(0, CAMERA_HEIGHT - this.mOnScreenControlBaseTextureRegion.getHeight(), this.mZoomCamera, this.mOnScreenControlBaseTextureRegion, this.mOnScreenControlKnobTextureRegion, 0.1f, 200, this.getVertexBufferObjectManager(), new IAnalogOnScreenControlListener() {
			@Override
			public void onControlChange(final BaseOnScreenControl pBaseOnScreenControl, final float pValueX, final float pValueY) {				
				driveState = -pValueX;
			}

			@Override
			public void onControlClick(final AnalogOnScreenControl pAnalogOnScreenControl) {
				Debug.d("onControlClick");
			}
		});
		analogOnScreenControl.getControlBase().setBlendFunction(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);
		analogOnScreenControl.getControlBase().setAlpha(0.5f);
		analogOnScreenControl.getControlBase().setScaleCenter(0, 128);
		analogOnScreenControl.getControlBase().setScale(1.25f);
		analogOnScreenControl.getControlKnob().setScale(1.25f);
		analogOnScreenControl.refreshControlKnobPosition();
		
		mScene.setChildScene(analogOnScreenControl);
		
	}

	private String readRawTextFile(int resId)
    {
         InputStream inputStream = getResources().openRawResource(resId);

            InputStreamReader inputreader = new InputStreamReader(inputStream);
            BufferedReader buffreader = new BufferedReader(inputreader);
             String line;
             StringBuilder text = new StringBuilder();

             try {
               while (( line = buffreader.readLine()) != null) {
                   text.append(line);
                 }
           } catch (IOException e) {
               return null;
           }
           
           return text.toString();
    }
	
	// ===========================================================
	// Inner and Anonymous Classes
	// ===========================================================
}
