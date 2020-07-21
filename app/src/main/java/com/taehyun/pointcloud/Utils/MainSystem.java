package com.taehyun.pointcloud.Utils;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.os.AsyncTask;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Toast;

import com.curvsurf.fsweb.FindSurfaceRequester;
import com.curvsurf.fsweb.RequestForm;
import com.curvsurf.fsweb.ResponseForm;
import com.google.ar.core.Frame;
import com.taehyun.pointcloud.Activity.MainActivity;
import com.taehyun.pointcloud.Model.Plane;
import com.taehyun.pointcloud.Renderer.PointCloudRenderer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.security.cert.PolicyNode;
import java.util.ArrayList;
import java.util.HashMap;

public class MainSystem {
    private String TAG = getClass().getName();
    private Context mContext;
    private static final String REQUEST_URL = "https://developers.curvsurf.com/FindSurface/plane"; // Plane searching server address

    private PointCloudRenderer pointCloudRenderer;
    private GLSurfaceView glView;
    private Frame frame;
    private ObjectFinder objectFinder = new ObjectFinder();
    private ObjectGroundFinder objectGroundFinder = new ObjectGroundFinder();


    private float epsilon = 0.01f;
    private float[] ray = null;
    private float circleRad = 0.25f;
    private float z_dis = 0;

    private float[] groundNorm;
    private float groundD;
    private float boxHeight = 0.f;
    HashMap<Integer, float[]> pointsMap = new HashMap<>();
    private int boxHeightId;


    public float[][] boxPoints;

    public MainSystem(Context context, PointCloudRenderer pointCloudRenderer, GLSurfaceView glView){
        this.pointCloudRenderer = pointCloudRenderer;
        this.glView = glView;
        this.mContext = context;
    }



    public View.OnTouchListener pickGround = new View.OnTouchListener() {
        @Override
        public boolean onTouch(View v, MotionEvent event) {
            float tx = event.getX();
            float ty = event.getY();//화면상의 위치
            // ray 생성
            frame = SingleTonClass.getInstance().frame;
            ray = screenPointToWorldRay(tx, ty, frame);
            float[] rayDest = new float[]{
                    ray[0]+ray[3],
                    ray[1]+ray[4],
                    ray[2]+ray[5],
            };
            float[] rayUnit = new float[] {ray[3],ray[4],ray[5]};
            pointCloudRenderer.pickPoint(ray, rayUnit);

            z_dis = pointCloudRenderer.getSeedArr()[2];

            if(objectFinder != null){
                if(objectFinder.getStatus() == AsyncTask.Status.FINISHED || objectFinder.getStatus() == AsyncTask.Status.RUNNING){
                    objectFinder.cancel(true);
                    objectFinder = new ObjectFinder();
                }
                objectFinder.execute();
            }
            return false;
        }
    };



    private void FindObject(float[] ul, float[] ur, float[] ll, float[] lr){
        float[] vec1 = VectorCal.sub(lr, ll);
        float[] vec2 = VectorCal.sub(ul, ll);

        groundNorm = VectorCal.outer(vec1,vec2);
        VectorCal.normalize(groundNorm);
        groundD =
                -groundNorm[0]*ul[0] - groundNorm[1]*ul[1] - groundNorm[2]*ul[2];



        HashMap<Integer, float[]> tmp;
        tmp = pointCloudRenderer.filteredPoints;
        for(int id : tmp.keySet())
        {
            float[] point = tmp.get(id);
            float a = VectorCal.inner(groundNorm,point);

            if(a < -groundD-epsilon || a> -groundD+epsilon)
            {
                pointCloudRenderer.objectPoints.put(id,point);
            }
        }
    }




    public float[] screenPointToWorldRay(float xPx, float yPx, Frame frame) {		// pointCloudActivity, 이름 그대로 화면 터치부분에서 월드 스페이스 ray로
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

    public class ObjectFinder extends AsyncTask<Object, ResponseForm.PlaneParam, ResponseForm.PlaneParam> {
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
                FindObject(o. ul, o. ur, o. ll, o. lr);
                float a = groundNorm[0];
                float b = groundNorm[1];
                float c = groundNorm[2];
                float d = groundD;

                //오브젝트의 높이 구함
                for(int id : pointCloudRenderer.objectPoints.keySet()) {
                    float[] p = pointCloudRenderer.objectPoints.get(id);
                    double distance = Math.abs(a*p[0] + b*p[1] + c*p[2] + d) / Math.sqrt(a*a + b*b +c*c);
                    if(distance > boxHeight){
                        boxHeight = (float)distance;
                        boxHeightId = id;
                    }

                    p[0] = p[0] - (a * boxHeight);
                    p[1] = p[1] - (b * boxHeight);
                    p[2] = p[2] - (c * boxHeight);

                    pointsMap.put(id, p);
                }

                if(objectGroundFinder != null){
                    if(objectGroundFinder.getStatus() == AsyncTask.Status.FINISHED || objectGroundFinder.getStatus() == AsyncTask.Status.RUNNING){
                        objectGroundFinder.cancel(true);
                        objectGroundFinder = new ObjectGroundFinder();
                    }
                    objectGroundFinder.execute();
                }

            }catch (Exception e){
                Log.d("Plane", e.getMessage());
            }
            if(o == null){
                Toast.makeText(mContext, "평면 추출 실패",Toast.LENGTH_SHORT).show();
            }
        }
    }





    public class ObjectGroundFinder extends AsyncTask<Object, ResponseForm.PlaneParam, ResponseForm.PlaneParam> {
        @Override
        protected ResponseForm.PlaneParam doInBackground(Object[] objects) {
            // Ready Point Cloud
            ByteBuffer bb_object = ByteBuffer.allocateDirect(4 * 4 * pointsMap.size());
            bb_object.order(ByteOrder.nativeOrder());
            FloatBuffer points = bb_object.asFloatBuffer();
            for(int id : pointsMap.keySet()){
                float[] tmp = new float[4];
                tmp[0] = pointsMap.get(id)[0];
                tmp[1] = pointsMap.get(id)[1];
                tmp[2] = pointsMap.get(id)[2];
                tmp[3] = 1.0f;
                points.put(tmp);
            }
            points.position(0);

            // Ready Request Form
            RequestForm rf = new RequestForm();

            rf.setPointBufferDescription(points.capacity()/4, 16, 0); //pointcount, pointstride, pointoffset
            rf.setPointDataDescription(0.05f, 0.01f); //accuracy, meanDistance
            rf.setTargetROI(boxHeightId, Math.max(z_dis * circleRad, 0.1f));//seedIndex,touchRadius
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
                float[] lul = o.ul;
                float[] lur = o.ur;
                float[] lll = o.ll;
                float[] llr = o.lr;

                float[] uul = new float[]{
                        lul[0] + (groundNorm[0] * boxHeight),
                        lul[1] + (groundNorm[1] * boxHeight),
                        lul[2] + (groundNorm[2] * boxHeight)
                };

                float[] uur = new float[]{
                        lur[0] + (groundNorm[0] * boxHeight),
                        lur[1] + (groundNorm[1] * boxHeight),
                        lur[2] + (groundNorm[2] * boxHeight)
                };
                float[] ull = new float[]{
                        lll[0] + (groundNorm[0] * boxHeight),
                        lll[1] + (groundNorm[1] * boxHeight),
                        lll[2] + (groundNorm[2] * boxHeight)
                };

                float[] ulr = new float[]{
                        llr[0] + (groundNorm[0] * boxHeight),
                        llr[1] + (groundNorm[1] * boxHeight),
                        llr[2] + (groundNorm[2] * boxHeight)
                };

                boxPoints = new float[][]{
                    ull, lll, ulr, ulr, lll, llr,
                        ulr, llr, uur, uur, llr, lur,
                        uur, lur, uul, uul, lur, lul,
                        uul, lul, ull, ull, lul, lll,
                        uul, ull, uur, uur, ull, ulr,
                        lll, lul, llr, llr, lul, lur
                };

            }catch (Exception e){
                Log.d("Plane", e.getMessage());
            }
            if(o == null){
                Toast.makeText(mContext, "objectgroundfinder failed",Toast.LENGTH_SHORT).show();

            }
        }
    }

    public boolean isBoxReady(){
        if(boxPoints != null && boxPoints.length >= 6)
            return true;
        return false;
    }
}
