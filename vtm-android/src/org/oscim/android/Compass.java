/**
 * Created by github.com/MarsVard on 19/08/14.
 * compass class with callbacks and listeners
 *
 * This file is part of the OpenScienceMap project (http://www.opensciencemap.org).
 *
 * This program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */
package org.oscim.android;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.oscim.map.Map;
public class Compass implements SensorEventListener {

    private final SensorManager mSensorManager;
    private final Sensor mAccelerometer;
    private final Sensor mMagnetometer;
    private final Context ctx;
    private Map mMap = null;
    private CompassUpdateListener compassUpdateListener;

    private float[] mLastAccelerometer = new float[9];
    private float[] mLastMagnetometer = new float[9];

    private boolean mLastAccelerometerSet = false;
    private boolean mLastMagnetometerSet = false;

    private float[] mR = new float[16];
    private float[] mOrientation = new float[3];

    public Compass(Context ctx, Map map) {
        this(ctx);
        this.mMap = map;
    }

    public Compass(Context ctx) {
        this.ctx = ctx;
        // get SensorManager from Context
        mSensorManager = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);

        // get Sensors
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    }

    /**
     * Enable sensor listeners, this should be called in onResume
     */
    public void enable() {
        // register sensors and this(listener)
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, mMagnetometer, SensorManager.SENSOR_DELAY_UI);

        mLastAccelerometerSet = false;
        mLastMagnetometerSet = false;
    }

    /**
     * Disable sensor listeners, this should be called in onPause
     */
    public void disable() {
        // unregister listener
        mSensorManager.unregisterListener(this);
    }

    /**
     * get Compass angle in degrees
     *
     * @return float degrees angle
     */
    public float getAngleDegrees() {
        float azimuthInRadians = mOrientation[0];
        float azimuthInDegress = (float) Math.toDegrees(azimuthInRadians);
        if (azimuthInDegress < 0.0f) {
            azimuthInDegress += 360.0f;
        }
        return azimuthInDegress;
    }

    /**
     * set map that needs to be updated according to compass
     *
     * @param mMap
     */
    public void setMap(Map mMap) {
        this.mMap = mMap;
    }

    /**
     * unset map, disabling automatic updates to viewport
     */
    public void unsetMap() {
        this.mMap = null;
    }

    /**
     * set compass update listener to receive azimuth, pitch, roll values
     *
     * @param compassUpdateListener
     */
    public void setCompassUpdateListener(CompassUpdateListener compassUpdateListener) {
        this.compassUpdateListener = compassUpdateListener;
    }

    /**
     * unset compassUpdateListener
     */
    public void removeCompassUpdateListener() {
        this.compassUpdateListener = null;
    }

    // callback interface to listen for updates outside this class
    public interface CompassUpdateListener {
        public void compassUpdateReceived(float azimuth, float pitch, float roll);
    }

    //
    // sensor event listeners
    //
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        // clone sensor data so it doesn't change while we are using it
        float[] sensorValues = sensorEvent.values.clone();

        // check sensore type
        // store latest sensor data locally
        if (sensorEvent.sensor == mAccelerometer) {
            lowPass(sensorValues, mLastAccelerometer);

            mLastAccelerometerSet = true;
        } else if (sensorEvent.sensor == mMagnetometer) {
            lowPass(sensorValues, mLastMagnetometer);

            mLastMagnetometerSet = true;
        } else {
            // oops, wrong sensor
            return;
        }

        // if both sensor data has been received measure orientation
        if (mLastAccelerometerSet && mLastMagnetometerSet) {
            boolean success = SensorManager.getRotationMatrix(mR, null, mLastAccelerometer, mLastMagnetometer);
            if (!success)
                return; // failed to get RotationMatrix, abort mission

            if (sensorEvent.sensor == mAccelerometer) {
                // fix orientation of sensor values
                switch (((WindowManager) ctx.getSystemService(Context.WINDOW_SERVICE)).getDefaultDisplay().getRotation()) {
                    case Surface.ROTATION_0:
                        SensorManager.remapCoordinateSystem(mR, SensorManager.AXIS_X, SensorManager.AXIS_Y, mR);
                        break;

                    case Surface.ROTATION_90:
                        SensorManager.remapCoordinateSystem(mR, SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_X, mR);
                        break;

                    case Surface.ROTATION_180:
                        SensorManager.remapCoordinateSystem(mR, SensorManager.AXIS_MINUS_X, SensorManager.AXIS_MINUS_Y, mR);
                        break;

                    case Surface.ROTATION_270:
                        SensorManager.remapCoordinateSystem(mR, SensorManager.AXIS_MINUS_Y, SensorManager.AXIS_X, mR);
                        break;
                }
            }

            // get final orientation values
            SensorManager.getOrientation(mR, mOrientation);

            // update map if needed
            if (mMap != null) {
                float mapAngle = mMap.getMapPosition().getBearing();
                if (Math.abs(mapAngle - getAngleDegrees()) > 0.25) {
                    mMap.viewport().setRotation(-getAngleDegrees());
                    mMap.updateMap(true);
                }
            }

            // report compass orientation change
            if (compassUpdateListener != null) {
                compassUpdateListener.compassUpdateReceived(mOrientation[0], mOrientation[1], mOrientation[2]);
            }
        }


    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    /*
     * copied from http://stackoverflow.com/questions/4611599/help-smoothing-data-from-a-sensor/5780505#5780505
     * time smoothing constant for low-pass filter
     * 0 ≤ α ≤ 1 ; a smaller value basically means more smoothing
     * See: http://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
     *
     * @see http://en.wikipedia.org/wiki/Low-pass_filter#Algorithmic_implementation
     * @see http://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
     */
    static final float ALPHA = 0.1f;

    protected float[] lowPass(float[] input, float[] output) {
        if (output == null) return input;

        for (int i = 0; i < input.length; i++) {
            output[i] = (input[i] * ALPHA) + (output[i] * (1.0f - ALPHA));
        }
        return output;
    }
}
