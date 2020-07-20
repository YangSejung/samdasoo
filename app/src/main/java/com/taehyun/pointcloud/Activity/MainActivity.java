package com.taehyun.pointcloud.Activity;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.pm.PackageManager;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Debug;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.curvsurf.fsweb.FindSurfaceRequester;
import com.curvsurf.fsweb.RequestForm;
import com.curvsurf.fsweb.ResponseForm;
import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Camera;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.taehyun.pointcloud.Model.Plane;
import com.taehyun.pointcloud.R;
import com.taehyun.pointcloud.Renderer.BackgroundRenderer;
import com.taehyun.pointcloud.Renderer.LineRenderer;
import com.taehyun.pointcloud.Renderer.PlaneRenderer;
import com.taehyun.pointcloud.Renderer.PointCloudRenderer;
import com.taehyun.pointcloud.Utils.VectorCal;


import java.io.Console;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class MainActivity extends AppCompatActivity implements GLSurfaceView.Renderer{
    String TAG = getClass().getName();

    private boolean mUserRequestedInstall = false;//다운 요청 할 때 사용
    private boolean mViewportChanged = false;//가로 세로 화면 전환할 때 사용
    private int mViewportWidth = -1;//가로 세로 화면 전환할 때 사용
    private int mViewportHeight = -1;//가로 세로 화면 전환할 때 사용

    private float epsilon = 0.20f;

    //카메라 권한
    private String[] REQUIRED_PERMISSSIONS = {Manifest.permission.CAMERA};//전체 권한을 받는 배열
    private final int PERMISSION_REQUEST_CODE = 0; // PROTECTION_NORMAL

    private GLSurfaceView glView;
    BackgroundRenderer backgroundRenderer = new BackgroundRenderer();
    PointCloudRenderer pointCloudRenderer = new PointCloudRenderer();
    LineRenderer lineRenderer = new LineRenderer();
    private Session session;
    private Frame frame;//클래스들 정의

    private Button btn_record;

    private Button btn_debug;

    private boolean recording = false;
    private int renderingMode = 0;  // 0:start, 1:recording, 2:recorded 3:pickPoint

    private float[] viewMatrix = new float[16];//매트릭스들 정의
    private float[] projMatrix = new float[16];
    private float[] vpMatrix = new float[16];//p*v 매트릭스

    private float[] ray = null;
    private static final String REQUEST_URL = "https://developers.curvsurf.com/FindSurface/plane"; // Plane searching server address
    private planeFinder myPlaneFinder = new planeFinder();
    private PlaneRenderer planeRenderer = new PlaneRenderer();
    private float[] pVertex = null;
    private Plane plane = null;

    private float circleRad = 0.25f;
    private float z_dis = 0;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        glView = findViewById(R.id.glView);

        glView.setPreserveEGLContextOnPause(true);//이건 뭐니

        glView.setEGLContextClientVersion(2);
        glView.setEGLConfigChooser(8,8,8,8,16,0);
        glView.setRenderer(this);
        glView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);


        btn_debug = findViewById(R.id.button1);
        btn_debug.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                findPlane(plane.getUl(), plane.getUr(), plane.getLl(), plane.getLr());
                renderingMode = 4;
            }
        });

        glView.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                float tx = event.getX();
                float ty = event.getY();//화면상의 위치
                // ray 생성
                ray = screenPointToWorldRay(tx, ty, frame);
                float[] rayDest = new float[]{
                        ray[0]+ray[3],
                        ray[1]+ray[4],
                        ray[2]+ray[5],
                };
                lineRenderer.bufferUpdate(ray, rayDest);
                float[] rayUnit = new float[] {ray[3],ray[4],ray[5]};
                pointCloudRenderer.pickPoint(ray, rayUnit);
                renderingMode = 3;

                z_dis = pointCloudRenderer.getSeedArr()[2];

                if(myPlaneFinder != null){
                    if(myPlaneFinder.getStatus() == AsyncTask.Status.FINISHED || myPlaneFinder.getStatus() == AsyncTask.Status.RUNNING){
                        myPlaneFinder.cancel(true);
                        myPlaneFinder = new planeFinder();
                    }
                    myPlaneFinder.execute();
                }
                return false;
            }
        });
        btn_record = findViewById(R.id.btn_record);
        btn_record.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                recording = !recording;
                if(recording) {
                    renderingMode = 1;
                    Toast.makeText(getApplicationContext(), "start recording", Toast.LENGTH_SHORT).show();
                    btn_record.setForeground(getApplicationContext().getDrawable(R.drawable.ic_recstop));
                }
                else {
                    if(renderingMode == 1) pointCloudRenderer.filterPoints();
                    renderingMode = 2;
                    Toast.makeText(getApplicationContext(), "stop recording", Toast.LENGTH_SHORT).show();
                    btn_record.setForeground(getApplicationContext().getDrawable(R.drawable.ic_recbutton));
                }
            }
        });

        for(String permission : REQUIRED_PERMISSSIONS){
            if(ContextCompat.checkSelfPermission(this, permission) != PackageManager.PERMISSION_GRANTED){
                ActivityCompat.requestPermissions(this, REQUIRED_PERMISSSIONS, PERMISSION_REQUEST_CODE);
            }
        }
    }
    @Override
    protected void onDestroy() {
        super.onDestroy();
        finish();
    }

    @Override
    protected void onPause() {
        super.onPause();
        if(session != null){
            glView.onPause();
            session.pause();
        }
    }
    @Override
    protected void onResume() {
        super.onResume();

        if(session == null){
            try{
                switch(ArCoreApk.getInstance().requestInstall(this,!mUserRequestedInstall)){
                    case INSTALL_REQUESTED:
                        mUserRequestedInstall = true;
                        return;
                    case INSTALLED:
                        break;
                }
                session = new Session(this);

                Config config = new Config(session);
                config.setLightEstimationMode(Config.LightEstimationMode.DISABLED);
                config.setPlaneFindingMode(Config.PlaneFindingMode.DISABLED);
                config.setFocusMode(Config.FocusMode.AUTO);
                session.configure(config);

            }catch (Exception e){
                Log.d("ULTRA", e.getMessage());
                return;
            }
        }

        try{
            session.resume();
        }catch (CameraNotAvailableException e){
            e.printStackTrace();
            session = null;
            finish();
        }

        glView.onResume();
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f);//처음 화면 색
        GLES20.glDisable(GLES20.GL_CULL_FACE);
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);//뒤에 가려진 면이 안나오도록
        try{
            backgroundRenderer.createOnGlThread(this);//이제 생성자를 여기서 부르지 않고 createOnGLThread를 씀
            pointCloudRenderer.createOnGlThread(this);//이제 생성자를 여기서 부르지 않고 createOnGLThread를 씀
            lineRenderer.createGlThread(this);//이제 생성자를 여기서 부르지 않고 createOnGLThread를 씀
            planeRenderer.createGlThread(this);//이제 생성자를 여기서 부르지 않고 createOnGLThread를 씀
        }catch (IOException e){
            e.getMessage();
        }
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        mViewportChanged = true;
        mViewportWidth = width;
        mViewportHeight = height;
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT|GLES20.GL_DEPTH_BUFFER_BIT);
        //MVPMatrix M : Model, V : view, P : projection
        // Local -> world -> camera -> projection
        // Model matrix는 4*4 표준 변환 행렬(스케일링, 이동, 회전 전부 포함)
        // 보통 스케일링, 회전, 트랜슬레이션 순으로 사용 (T*P*S)
        //
        // Local Matrix에 model Matrix를 곱함 -> world space
        // 여기에 view Matrix를 곱함 -> camera space
        // 여기에 projection Matrix를 곱함 -> 2D로 정교화
        // 만약 Model이 필요 없다면 vp 매트릭스가 최종인가?
        // ex)
        // projection     View        Model     Local
        // [         ][           ][          ] [ x  ]
        // |         ||           ||  T*R*S   | | y  |
        // |         ||           ||          | | z  |
        // [         ][           ][          ] [1.0f]

        if(session == null){
            return;
        }
        if(mViewportChanged){
            int displayRotation = getWindowManager().getDefaultDisplay().getRotation();
            session.setDisplayGeometry(displayRotation, mViewportWidth, mViewportHeight);
        }
        try{
            session.setCameraTextureName(backgroundRenderer.getTextureId());
            frame = session.update();
            Camera camera = frame.getCamera();
            backgroundRenderer.draw(frame);

            if(camera.getTrackingState() == TrackingState.TRACKING){
                camera.getViewMatrix(viewMatrix, 0);
                camera.getProjectionMatrix(projMatrix, 0, 0.1f,100.0f);
                Matrix.multiplyMM(vpMatrix,0, projMatrix, 0, viewMatrix,0);
                pointCloudRenderer.update(frame.acquirePointCloud(), recording);
                Log.d("RMode", String.format("%b %d", recording, renderingMode));
                switch (renderingMode){
                    case 0:
                        pointCloudRenderer.draw(viewMatrix, projMatrix);
                        break;
                    case 1:
                        pointCloudRenderer.draw_conf(viewMatrix, projMatrix);
                        break;
                    case 2:
                        pointCloudRenderer.draw_final(viewMatrix, projMatrix);
                        lineRenderer.setCircleVertex(circleRad);
                        lineRenderer.draw_circle(projMatrix);
                        Log.d("numPoints", String.valueOf(pointCloudRenderer.finalPointBuffer.remaining()));
                        break;
                    case 3:
                        pointCloudRenderer.draw_seedPoint(vpMatrix);
                        break;
                    case 4:
                        pointCloudRenderer.draw_objectPoint(viewMatrix, projMatrix);
                        break;
                }
                if(ray != null){
                    lineRenderer.draw(vpMatrix);
                }
                if(planeRenderer.planeVertex != null){
                    planeRenderer.draw(vpMatrix);
                    // grid
                    lineRenderer.bufferUpdate(plane.gridPoints[0],plane.gridPoints[4]);
                    lineRenderer.draw(vpMatrix);
                    lineRenderer.bufferUpdate(plane.gridPoints[4],plane.gridPoints[8]);
                    lineRenderer.draw(vpMatrix);
                    lineRenderer.bufferUpdate(plane.gridPoints[8],plane.gridPoints[12]);
                    lineRenderer.draw(vpMatrix);
                    lineRenderer.bufferUpdate(plane.gridPoints[12],plane.gridPoints[0]);
                    lineRenderer.draw(vpMatrix);

                    lineRenderer.bufferUpdate(plane.gridPoints[1],plane.gridPoints[11]);
                    lineRenderer.draw(vpMatrix);
                    lineRenderer.bufferUpdate(plane.gridPoints[2],plane.gridPoints[10]);
                    lineRenderer.draw(vpMatrix);
                    lineRenderer.bufferUpdate(plane.gridPoints[3],plane.gridPoints[9]);
                    lineRenderer.draw(vpMatrix);

                    lineRenderer.bufferUpdate(plane.gridPoints[5],plane.gridPoints[15]);
                    lineRenderer.draw(vpMatrix);
                    lineRenderer.bufferUpdate(plane.gridPoints[6],plane.gridPoints[14]);
                    lineRenderer.draw(vpMatrix);
                    lineRenderer.bufferUpdate(plane.gridPoints[7],plane.gridPoints[13]);
                    lineRenderer.draw(vpMatrix);
                }
            }

        }catch (CameraNotAvailableException e){
            finish();
        }
    }

    float[] screenPointToWorldRay(float xPx, float yPx, Frame frame) {		// pointCloudActivity, 이름 그대로 화면 터치부분에서 월드 스페이스 ray로
        //xPx : 터치한 x좌표, yPx : 터치한 y 좌표
        // ray[0~2] : camera pose
        // ray[3~5] : Unit vector of ray
        float[] ray_clip = new float[4];
        ray_clip[0] = 2.0f * xPx / glView.getMeasuredWidth() - 1.0f;
        // +y is up (android UI Y is down):
        ray_clip[1] = 1.0f - 2.0f * yPx / glView.getMeasuredHeight();
        ray_clip[2] = -1.0f; // +z is forwards (remember clip, not camera) 클립이 뭔 뜻일까
        ray_clip[3] = 1.0f; // w (homogenous coordinates)
        //위치 조절 하는 건가봐

        float[] ProMatrices = new float[32];  // {proj, inverse proj}
        frame.getCamera().getProjectionMatrix(ProMatrices, 0, 0.1f, 100.0f);
        Matrix.invertM(ProMatrices, 16, ProMatrices, 0);
        float[] ray_eye = new float[4];
        Matrix.multiplyMV(ray_eye, 0, ProMatrices, 16, ray_clip, 0);

        ray_eye[2] = -1.0f;
        ray_eye[3] = 0.0f;

        float[] out = new float[6];
        float[] ray_wor = new float[4];
        float[] ViewMatrices = new float[32];

        frame.getCamera().getViewMatrix(ViewMatrices, 0);
        Matrix.invertM(ViewMatrices, 16, ViewMatrices, 0);
        Matrix.multiplyMV(ray_wor, 0, ViewMatrices, 16, ray_eye, 0);

        float size = (float)Math.sqrt(ray_wor[0] * ray_wor[0] + ray_wor[1] * ray_wor[1] + ray_wor[2] * ray_wor[2]);

        out[3] = ray_wor[0] / size;
        out[4] = ray_wor[1] / size;
        out[5] = ray_wor[2] / size;

        out[0] = frame.getCamera().getPose().tx();
        out[1] = frame.getCamera().getPose().ty();
        out[2] = frame.getCamera().getPose().tz();

        return out;
    }
    public class planeFinder extends AsyncTask<Object, ResponseForm.PlaneParam, ResponseForm.PlaneParam> {
        @Override
        protected ResponseForm.PlaneParam doInBackground(Object[] objects) {
            // Ready Point Cloud
            FloatBuffer points = pointCloudRenderer.finalPointBuffer;

            // Ready Request Form
            RequestForm rf = new RequestForm();

            rf.setPointBufferDescription(points.capacity()/4, 16, 0); //pointcount, pointstride, pointoffset
            rf.setPointDataDescription(0.05f, 0.01f); //accuracy, meanDistance
            rf.setTargetROI(pointCloudRenderer.seedPointID, Math.max(z_dis * circleRad, 0.1f));//seedIndex,touchRadius
            rf.setAlgorithmParameter(RequestForm.SearchLevel.NORMAL, RequestForm.SearchLevel.NORMAL);//LatExt, RadExp
            Log.d("PointsBuffer", points.toString());
            FindSurfaceRequester fsr = new FindSurfaceRequester(REQUEST_URL, true);
            // Request Find Surface
            try{
                Log.d("PlaneFinder", "request");
                ResponseForm resp = fsr.request(rf, points);
                if(resp != null && resp.isSuccess()) {
                    ResponseForm.PlaneParam param = resp.getParamAsPlane();
                    Log.d("PlaneFinder", "request success");
                    return param;
                }
                else{
                    Log.d("PlaneFinder", "request fail");
                }
            }catch (Exception e){
                e.printStackTrace();
            }
            return null;
        }

        @Override
        protected void onPreExecute() {
            super.onPreExecute();
        }

        @Override
        protected void onPostExecute(ResponseForm.PlaneParam o) {
            super.onPostExecute(o);
            try{
                plane = new Plane(o.ll, o.lr, o.ur, o.ul, frame.getCamera());
                pVertex = plane.getPlaneVertex();
                planeRenderer.bufferUpdate(pVertex);
            }catch (Exception e){
                Log.d("Plane", e.getMessage());
            }
            if(o == null){
                Toast.makeText(getApplicationContext(), "평면 추출 실패",Toast.LENGTH_SHORT).show();
            }
        }
    }

    private void findPlane(float[] ul, float[] ur, float[] ll, float[] lr){
        float[] vec1 = VectorCal.sub(lr, ll);
        float[] vec2 = VectorCal.sub(ul, ll);

        float[] normalVector = VectorCal.outer(vec1,vec2);
        VectorCal.normalize(normalVector);

        HashMap<Integer, float[]> tmp;
        tmp = pointCloudRenderer.filteredPoints;
        for(int id : tmp.keySet())
        {
            float[] point = tmp.get(id);
            float a = VectorCal.inner(normalVector,point);

            if(a < -1-epsilon || a> -1+epsilon)
            {
                pointCloudRenderer.objectPoints.put(id,point);
            }

        }


            /*for (int id : allPoints.keySet()) {
                ArrayList<float[]> list = allPoints.get(id);
                float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
                for (float[] p : list) {
                    mean_x += p[0]; //  x
                    mean_y += p[1]; //  y
                    mean_z += p[2]; //  z
                }
                mean_z /= list.size();
                mean_x /= list.size();
                mean_y /= list.size();*/
    }

}
