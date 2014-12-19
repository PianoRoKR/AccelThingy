/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.accelerometerplay;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.BitmapFactory.Options;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.Toast;

import java.util.ArrayList;

/**
 * This is an example of using the accelerometer to integrate the device's
 * acceleration to a position using the Verlet method. This is illustrated with
 * a very simple particle system comprised of a few iron balls freely moving on
 * an inclined wooden table. The inclination of the virtual table is controlled
 * by the device's accelerometer.
 * 
 * @see SensorManager
 * @see SensorEvent
 * @see Sensor
 */

public class AccelerometerPlayActivity extends Activity {

    private SimulationView mSimulationView;
    private ViewGroup mLayout;
    private SensorManager mSensorManager;
    private PowerManager mPowerManager;
    private WindowManager mWindowManager;
    private Display mDisplay;
    private WakeLock mWakeLock;
    private Button mRestartButton;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Get an instance of the SensorManager
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        // Get an instance of the PowerManager
        mPowerManager = (PowerManager) getSystemService(POWER_SERVICE);

        // Get an instance of the WindowManager
        mWindowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
        mDisplay = mWindowManager.getDefaultDisplay();
        // Create a bright wake lock
        mWakeLock = mPowerManager.newWakeLock(PowerManager.SCREEN_BRIGHT_WAKE_LOCK, getClass()
                .getName());

        mRestartButton = new Button(getApplicationContext());
        mRestartButton.setText("Restart Game");
        mRestartButton.setTextColor(Color.parseColor("#000000"));
        mRestartButton.setTextSize(12);
        mRestartButton.setPadding(10,10,10,10);
        // instantiate our simulation view and set it as the activity's content
        mSimulationView = new SimulationView(this);
        mRestartButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mSimulationView.seeStuff(v.getContext());
            }
        });
        mSimulationView.setPadding(0,50,0,0);
        mLayout = new LinearLayout(getApplicationContext());
        ((LinearLayout)mLayout).setOrientation(LinearLayout.VERTICAL);
        mLayout.addView(mRestartButton);
        mLayout.addView(mSimulationView);
        setContentView(mLayout);
    }

    @Override
    protected void onResume() {
        super.onResume();
        /*
         * when the activity is resumed, we acquire a wake-lock so that the
         * screen stays on, since the user will likely not be fiddling with the
         * screen or buttons.
         */
        mWakeLock.acquire();

        // Start the simulation
        mSimulationView.startSimulation();
    }

    @Override
    protected void onPause() {
        super.onPause();
        /*
         * When the activity is paused, we make sure to stop the simulation,
         * release our sensor resources and wake locks
         */

        // Stop the simulation
        mSimulationView.stopSimulation();

        // and release our wake-lock
        mWakeLock.release();
    }

    class SimulationView extends View implements SensorEventListener {
        // diameter of the balls in meters
        private static final float sBallDiameter = 0.004f;
        private static final float sBallDiameter2 = sBallDiameter * sBallDiameter;
        private Paint mTextPaint;
        // friction of the virtual table and air
        private static final float sFriction = 0.1f;

        private Sensor mAccelerometer;
        private long mLastT;
        private float mLastDeltaT;

        private float mXDpi;
        private float mYDpi;
        private float mMetersToPixelsX;
        private float mMetersToPixelsY;
        private Bitmap mBitmap;
        private Bitmap mWood;
        private float mXOrigin;
        private float mYOrigin;
        private float mSensorX;
        private float mSensorY;
        private long mSensorTimeStamp;
        private long mCpuTimeStamp;
        private float mHorizontalBound;
        private float mVerticalBound;
        private final CellSystem mCellSystem;

        /*
         * Each of our particle holds its previous and current position, its
         * acceleration. for added realism each particle has its own friction
         * coefficient.
         */
        class Particle {
            private float mPosX;
            private float mPosY;
            private float mAccelX;
            private float mAccelY;
            private float mLastPosX;
            private float mLastPosY;
            private float mOneMinusFriction;

            Particle() {
                // make each particle a bit different by randomizing its
                // coefficient of friction
                mOneMinusFriction = 1.0f - sFriction;
            }

            public void computePhysics(float sx, float sy, float dT, float dTC) {
                // Force of gravity applied to our virtual object
                final float m = 1000.0f; // mass of our virtual object
                final float gx = -sx * m;
                final float gy = -sy * m;

                /*
                 * F = mA <=> A = F / m We could simplify the code by
                 * completely eliminating "m" (the mass) from all the equations,
                 * but it would hide the concepts from this sample code.
                 */
                final float invm = 1.0f / m;
                final float ax = gx * invm;
                final float ay = gy * invm;

                /*
                 * Time-corrected Verlet integration The position Verlet
                 * integrator is defined as x(t+t) = x(t) + x(t) - x(t-t) +
                 * a(t)t2 However, the above equation doesn't handle variable
                 * t very well, a time-corrected version is needed: x(t+t) =
                 * x(t) + (x(t) - x(t-t)) * (t/t_prev) + a(t)t2 We also add
                 * a simple friction term (f) to the equation: x(t+t) = x(t) +
                 * (1-f) * (x(t) - x(t-t)) * (t/t_prev) + a(t)t2
                 */
                final float dTdT = dT * dT;
                final float x = mPosX + mOneMinusFriction * dTC * (mPosX - mLastPosX) + mAccelX
                        * dTdT;
                final float y = mPosY + mOneMinusFriction * dTC * (mPosY - mLastPosY) + mAccelY
                        * dTdT;
                mLastPosX = mPosX;
                mLastPosY = mPosY;
                mPosX = x;
                mPosY = y;
                mAccelX = ax;
                mAccelY = ay;
            }

            /*
             * Resolving constraints and collisions with the Verlet integrator
             * can be very simple, we simply need to move a colliding or
             * constrained particle in such way that the constraint is
             * satisfied.
             */
            public void resolveCollisionWithBounds() {
                final float xmax = mHorizontalBound;
                final float ymax = mVerticalBound;
                final float x = mPosX;
                final float y = mPosY;
                if (x > xmax) {
                    mPosX = xmax;
                } else if (x < -xmax) {
                    mPosX = -xmax;
                }
                if (y > ymax) {
                    mPosY = ymax;
                } else if (y < -ymax) {
                    mPosY = -ymax;
                }
            }
        }

        class Cell {
            private boolean mFilled = false;
            private RectF mRect;
            private Paint mPaint;
            private float mX;
            private float mY;
            private float mWidth;
            private float mHeight;
            private boolean mInMaze;
            private boolean mIsEnd;
            private boolean updated;

            Cell(float aX, float aY, float aWidth, float aHeight) {
                mFilled = false;
                mRect = new RectF(aX, aY, aX + aWidth, aY + aHeight);
                mX = aX;
                mY = aY;
                mWidth = aWidth;
                mHeight = aHeight;
                mPaint = new Paint();
            }

            public void MakeWall() {
                mPaint.setColor(0xff3B2C20);
                mFilled = true;
            }

            public void MakeColored(int i) {
                if(i == -1)
                    if(mFilled == true)
                        mPaint.setColor(0xff3B2C20);
                    else
                        mPaint.setColor(0x00000000);
                else {
                    switch (i) {
                        case 0:
                            mPaint.setColor(0xffffff00);
                            break;
                        case 1:
                            mPaint.setColor(0xffff00ff);
                            break;
                        case 2:
                            mPaint.setColor(0xff00ffff);
                            break;
                    }
                }
            }

            public void MakeOpening()
            {
                mPaint.setColor(0x00000000);
                mFilled = false;
            }
            public void Draw(Canvas canvas)
            {
                canvas.drawRect(mRect, mPaint);
            }
        }
		
		/*
		 * Prim's algorithm used to generate a random maze.
		 * Implementation based off Johnathan Zong's implementation of Prim's algorith
		 * http://jonathanzong.com/blog/2012/11/06/maze-generation-with-prims-algorithm
		 */
		class Primmy
		{
			public void MakeMaze()
			{
				//dimensions of the generated maze
				int r = 5;
				int c = 5;
				
				//Bulid maze and initialize with only walls
				StringBuilder s = new StringBuilder(w);
				for(int i = 0; i < c; i++)
					s.append('*');
				char[][] maze = new char[r][c];
				
				for(int i = 0; i < h; i++)
					maze[i] = s.toString().toCharArray();
				
				
				//Select a random point and open as start node
				// MakeOpening();?
				Point start = new Point((int)(Math.random() * r), (int)(Math.random()*c), null);
				maze[start.r][start.c] = 'S';
				
				//iterate through direct neighbors of node
				ArrayList<Point> area = new ArrayList<Point>();
				for(int i = -1; i <= 1; i++)
					for(int j = -1; j <= 1; j++)
					{
						if((i == 0 && j == 0) || (x != 0 && j != 0))
							continue;
						try
						{
							if(maze[start.r][start.c] == '.') 
								continue;
						}
						catch (Exception e)
						{
							continue;
						}
						//add eligible points
						area.add(new Point(start.r + i, start.c + j, start);
					}
					
					Point last = null;
					
					while(!area.isEmpty())
					{
						//pick current node at random
						Point cu = area.remove((int)(Math.random() * area.size()));
						Point op = cu.opposite();
						try
						{
							//if both node and its opposite are walls
							if(maze[cu.r][cu.c] == '*')
							{
								if(maze[cu.r][cu.c] == '*')
								{
									//open path between nodes
									//drawPath
									maze[cu.r][cu.c] = '.';
									maze[op.r][cu.c] = '.';
									
									//store last node in order to mark it laster
									last = op;
									
									//iterate through direct neighbors of node
									for(int i = -1; i <= 1; i++)
										for(int j = -1; j <= 1; j++)
										{
											if((i == 0 && j == 0) || (x != 0 && j != 0))
												continue;
											try
											{
												if(maze[op.r + i][op.c + j] == '.') 
													continue;
											}
											catch (Exception e)
											{
												continue;
											}
											//add eligible points
											area.add(new Point(op.r + i, op.c + j, start);
										}
								}
							}
						
						}
						catch(Exception e)
						{
							//ignore
						}
					}
					
					//if algorithm has resolved, mark the end
					if(area.isEmpty())
						maze[last.r][last.c] = 'E';
					
					//print final maze
					for(int i = 0; i < r; i++)
						for(int j = 0; j < c; j++)
							System.out.print(maze[i][j]);
						System.our.println();
			}
		}
		
		
		//Throw away class for current Prim implementation.  Can easily convert to a cell.
		static class Point
		{
			Integer r;
			Integer c;
			Point parent;
			
			public Point(int x, int y, Point p)
			{
				this.r = x;
				this.c = y;
				this.p = parent;
			}
			
			public Point opposite()
			{
				if(this.r.compareTo(parent.r) != 0)
					return new Point (this.r + this.r.compareTo(parent.r), this.c, this);
				if(this.c.compareTo(parent.c) != 0)
					return new Point (this.r, this.c + this.c.compareTo(parent.c), this);
				return null;
			}
		}

        /*
         * A particle system is just a collection of particles
         */
        class CellSystem {
            static final int NUM_PARTICLES = 1;
            private Particle mBall;
            public Cell[][] mCellArray;
            private int mWidth;
            private int mHeight;
            private float mBallH;
            private float mBallW;

            CellSystem(float width, float height, float dx, float dy) {
                /*
                 * Initially our particles have no speed or acceleration
                 */
                mBall = new Particle();
                mWidth = (int) (width / dx);
                mBallW = dx;
                mBallH = dy;
                mHeight = (int) (height / dy);
                mCellArray = new Cell[mWidth][mHeight];
                for (int i = 0; i < mWidth; i++) {
                    for (int j = 0; j < mHeight; j++) {
                        mCellArray[i][j] = new Cell(i * dx, j * dx, dx, dy);
                        if (i == mWidth - 1 || i == 0)
                            mCellArray[i][j].MakeWall();
                        else if (j == mHeight - 1 || j == 0)
                            mCellArray[i][j].MakeWall();
                        else
                            mCellArray[i][j].MakeOpening();
                    }
                }
            }

            /*
             * Update the position of each particle in the system using the
             * Verlet integrator.
             */
            private void updatePositions(float sx, float sy, long timestamp) {
                final long t = timestamp;
                if (mLastT != 0) {
                    final float dT = (float) (t - mLastT) * (1.0f / (1000000000.0f * 1.5f));
                    if (mLastDeltaT != 0) {
                        final float dTC = dT / mLastDeltaT;
                        mBall.computePhysics(sx/2, sy/2, dT, dTC);
                    }
                    mLastDeltaT = dT;
                }
                mLastT = t;
            }

            public void reset(float x, float y) {
                mBall.mPosX = x;
                mBall.mPosY = y;
                mBall.mAccelX = 0;
                mBall.mAccelY = 0;
                mBall.mLastPosX = x;
                mBall.mLastPosY = y;
                mBall.resolveCollisionWithBounds();
                for(Cell[] clist : mCellArray)
                    for(Cell c : clist) {
                        c.MakeColored(-1);
                    }
            }

            /*
             * Performs one iteration of the simulation. First updating the
             * position of all the particles and resolving the constraints and
             * collisions.
             */
            public void update(float sx, float sy, long now, float xc, float yc, float xs, float ys) {
                // update the system's positions
                updatePositions(sx, sy, now);
                mBall.resolveCollisionWithBounds();
                // We do no more than a limited number of iterations
                final int NUM_MAX_ITERATIONS = 10;
                float x = xc + (mBall.mPosX*xs + sBallDiameter / 2*xs);
                float y = yc - (mBall.mPosY*ys - sBallDiameter / 2*ys);
                int bx = (int) (x / mBallW);
                int by = (int) (y / mBallH);
                if(bx < 0)
                    bx = 0;
                if(by < 0)
                    by = 0;
                if(bx > mCellArray.length - 1)
                    bx = mCellArray.length - 1;
                if(by > mCellArray[bx].length - 1)
                    by = mCellArray.length - 1;
                float dx;
                float dy;
                Cell cell;
                int[] indexX = {-1, 0, 0, 1, 0};
                int[] indexY = {0, 1, -1, 0, 0};
                for (int index = 0; index < 4; index++) {
                    int i = indexX[index];
                    int j = indexY[index];
                    if (i == 0 && j == 0)
                        continue;
                    try {
                        cell = mCellArray[bx + i][by + j];
                    } catch (Exception e) {
                        continue;
                    }
                    if (!cell.mFilled)
                        continue;

                    boolean moveX = false;
                    boolean moveY = false;
                    float intersectX = 0;
                    float intersectY = 0;
                    switch (i) {
                        case -1:
                            moveX = cell.mX + cell.mWidth + sBallDiameter / 2 * xs > x;
                            break;
                        case 1:
                            moveX = cell.mX - sBallDiameter / 2 * xs < x;
                            break;
                    }
                    switch (j) {
                        case -1:
                            moveY = cell.mY + cell.mHeight + sBallDiameter / 2 * ys > y;
                            break;
                        case 1:
                            moveY = cell.mY - sBallDiameter / 2 * ys < y;
                            break;
                    }
                    switch (i) {
                        case -1:
                        case 1:
                            switch (j) {
                                case 0:
                                    if (moveX) {
                                        mBall.mPosX = mBall.mLastPosX;
                                        mBall.mAccelX = 0;
                                    }
                                    break;
                            }
                            break;
                        case 0:
                            switch (j) {
                                case -1:
                                case 1:
                                    if (moveY) {
                                        mBall.mPosY = mBall.mLastPosY;
                                        mBall.mAccelY = 0;
                                    }
                                    break;
                                case 0:
                                    mBall.mPosY = mBall.mLastPosY;
                                    mBall.mPosX = mBall.mLastPosX;
                                    mBall.mAccelX = 0;
                                    mBall.mAccelY = 0;
                                    break;
                            }
                            break;
                    }
                }
            }

            private float clamp(float val, float min, float max) {
                if(val < min)
                    return min;
                else if(val > max)
                    return max;
                else
                    return val;
            }

            public float getPosX() {
                return mBall.mPosX;
            }

            public float getPosY() {
                return mBall.mPosY;
            }

            /*public ArrayList<Cell> getNeighbors(float x, float y)
            {
                ArrayList<Cell> retval = new ArrayList<Cell>();
                int bx = (int)(x / mBallW);
                int by = (int)(y / mBallH);
                if(bx >= mCellArray.length || by >= mCellArray[0].length)
                    return retval;
                if(bx < 0 || by < 0)
                    return retval;

                return retval;
            }*/

            public void draw(Canvas canvas) {
                for (int i = 0; i < mCellArray.length; i++) {
                    for (int j = 0; j < mCellArray[i].length; j++) {
                            mCellArray[i][j].Draw(canvas);
                    }
                }
            }

        }

        public void startSimulation() {
            /*
             * It is not necessary to get accelerometer events at a very high
             * rate, by using a slower rate (SENSOR_DELAY_UI), we get an
             * automatic low-pass filter, which "extracts" the gravity component
             * of the acceleration. As an added benefit, we use less power and
             * CPU resources.
             */
            mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);
        }

        public void stopSimulation() {
            mSensorManager.unregisterListener(this);
        }

        public SimulationView(Context context) {
            super(context);


            mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);


            DisplayMetrics metrics = new DisplayMetrics();
            getWindowManager().getDefaultDisplay().getMetrics(metrics);
            mXDpi = metrics.xdpi;
            mYDpi = metrics.ydpi;
            mMetersToPixelsX = mXDpi / 0.0254f;
            mMetersToPixelsY = mYDpi / 0.0254f;

            // rescale the ball so it's about 0.5 cm on screen
            Bitmap ball = BitmapFactory.decodeResource(getResources(), R.drawable.ball);
            final int dstWidth = (int) (sBallDiameter * mMetersToPixelsX + 0.5f);
            final int dstHeight = (int) (sBallDiameter * mMetersToPixelsY + 0.5f);
            mBitmap = Bitmap.createScaledBitmap(ball, dstWidth, dstHeight, true);

            Options opts = new Options();
            opts.inDither = true;
            opts.inPreferredConfig = Bitmap.Config.RGB_565;
            mWood = BitmapFactory.decodeResource(getResources(), R.drawable.wood, opts);
            mCellSystem = new CellSystem(mWood.getWidth()-100, mWood.getHeight() - 100, dstWidth, dstHeight);
            mTextPaint = new Paint();
            mTextPaint.setColor(Color.parseColor("white"));
            mTextPaint.setTextSize(12);
        }

        public void seeStuff(Context context) {
            this.stopSimulation();
            mCellSystem.reset(0, 0);
            this.startSimulation();
        }

        @Override
        protected void onSizeChanged(int w, int h, int oldw, int oldh) {
            // compute the origin of the screen relative to the origin of
            // the bitmap
            mXOrigin = (w - mBitmap.getWidth()) * 0.5f;
            mYOrigin = (h - mBitmap.getHeight()) * 0.5f;
            mHorizontalBound = ((w / mMetersToPixelsX - sBallDiameter * 2f) * 0.5f);
            mVerticalBound = ((h / mMetersToPixelsY - sBallDiameter * 2f) * 0.5f);
        }

        @Override
        public void onSensorChanged(SensorEvent event) {
            if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
                return;
            /*
             * record the accelerometer data, the event's timestamp as well as
             * the current time. The latter is needed so we can calculate the
             * "present" time during rendering. In this application, we need to
             * take into account how the screen is rotated with respect to the
             * sensors (which always return data in a coordinate space aligned
             * to with the screen in its native orientation).
             */

            switch (mDisplay.getRotation()) {
                case Surface.ROTATION_0:
                    mSensorX = event.values[0];
                    mSensorY = event.values[1];
                    break;
                case Surface.ROTATION_90:
                    mSensorX = -event.values[1];
                    mSensorY = event.values[0];
                    break;
                case Surface.ROTATION_180:
                    mSensorX = -event.values[0];
                    mSensorY = -event.values[1];
                    break;
                case Surface.ROTATION_270:
                    mSensorX = event.values[1];
                    mSensorY = -event.values[0];
                    break;
            }

            mSensorTimeStamp = event.timestamp;
            mCpuTimeStamp = System.nanoTime();
        }

        @Override
        protected void onDraw(Canvas canvas) {

            /*
             * draw the background
             */

            canvas.drawBitmap(mWood, 0, 0, null);
            /*
             * compute the new position of our object, based on accelerometer
             * data and present time.
             */

            final CellSystem cellSystem = mCellSystem;
            final long now = mSensorTimeStamp + (System.nanoTime() - mCpuTimeStamp);
            final float sx = mSensorX;
            final float sy = mSensorY;

            final float xc = mXOrigin;
            final float yc = mYOrigin;
            final float xs = mMetersToPixelsX;
            final float ys = mMetersToPixelsY;

            cellSystem.update(sx/2, sy/2, now, xc, yc, xs, ys);
            cellSystem.draw(canvas);
            final Bitmap bitmap = mBitmap;

            final float x = xc + cellSystem.getPosX() * xs;
            final float y = yc - cellSystem.getPosY() * ys;
            canvas.drawBitmap(bitmap, x, y, null);
            String text = "X:" + String.format("%.2f", x) + " Y:" + String.format("%.2f", y);
            canvas.drawText(text, 0, text.length() - 1, x + sBallDiameter * xs * 1.01f, y + sBallDiameter * ys * 1.01f, mTextPaint);
            // and make sure to redraw asap
            invalidate();
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }
    }
}
