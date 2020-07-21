package com.taehyun.pointcloud.Renderer;

import android.content.Context;
import android.opengl.GLES20;

import com.taehyun.pointcloud.Utils.ShaderUtil;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

public class BoxRenderer {
    private static final String VERTEX_SHADER_NAME = "box.vert";
    private static final String FRAGMENT_SHADER_NAME = "box.frag";
    private int vertexShader;
    private int fragmentShader;

    private int mProgram;

    private int mPosition;
    private int mColor_u;
    private int uMVPMatrixHandle;

    private static final int COORDS_PER_VERTEX = 3;
    private static final int FLOAT_SIZE = 4;

    private FloatBuffer vertexBuffer;
    private ShortBuffer drawListBuffer;
    private FloatBuffer colorBuffer;

    public float[] boxVertex = null;
    private short drawOrder[] = { 0, 1, 2, 0, 2, 3 };
    private float[] colors = new float[]{
            0.0f, 0.0f, 1.0f, 0.2f,
            0.0f, 0.0f, 1.0f, 0.2f,
            0.0f, 0.0f, 1.0f, 0.2f,
            0.0f, 0.0f, 1.0f, 0.2f
    };

    public void bufferUpdate(float[][] box){
        boxVertex = new float[6 * 6 * 3];
        for(int i = 0; i < box.length; i++){
            for(int j = 0; j < box[i].length; j++){
                boxVertex[i*box[i].length + j] = box[i][j];
            }
        }

        ByteBuffer bb = ByteBuffer.allocateDirect(boxVertex.length * FLOAT_SIZE);//vertex를 float 버퍼로 바꾸는 과정, 항상 Direct로
        bb.order(ByteOrder.nativeOrder());//LSB MSB 이런거? 최상위 비트 어쩌구 자동으로
        vertexBuffer = bb.asFloatBuffer();//플로트 버퍼로써 사용하겠다.
        vertexBuffer.put(boxVertex);//좌표가 vertexBuffer로 들어감.
        vertexBuffer.position(0);//다 쓴 뒤 위치를 0으로

        ByteBuffer dlb = ByteBuffer.allocateDirect(drawOrder.length * 2); // (# of coordinate values * 2 bytes per short) 위와 같다.
        dlb.order(ByteOrder.nativeOrder());//drawList가 어디다가 쓰는건지 알아내면 위와 같음
        drawListBuffer = dlb.asShortBuffer();//drawElement할때 순서 정해줌.
        drawListBuffer.put(drawOrder);
        drawListBuffer.position(0);

        ByteBuffer cbb = ByteBuffer.allocateDirect(colors.length * FLOAT_SIZE);//이건 색을 보관하는 버퍼
        cbb.order(ByteOrder.nativeOrder());
        colorBuffer = cbb.asFloatBuffer();
        colorBuffer.put(colors);
        colorBuffer.position(0);
    }

    public void createGlThread(Context context) throws IOException {
        vertexShader = ShaderUtil.loadGLShader("Plane", context, GLES20.GL_VERTEX_SHADER, VERTEX_SHADER_NAME);//이게 이렇게 써도 작동이 되네; 어떤건지 어떻게 찾는거지
        fragmentShader = ShaderUtil.loadGLShader("Plane", context, GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER_NAME);//이게 이렇게 써도 작동이 되네 ㄷ;

        //bind shader's variable position
        mProgram = GLES20.glCreateProgram();
        GLES20.glAttachShader(mProgram, vertexShader);
        GLES20.glAttachShader(mProgram, fragmentShader);

        GLES20.glLinkProgram(mProgram);

        GLES20.glUseProgram(mProgram);//삼각형에선 draw에 있던데

        mPosition = GLES20.glGetAttribLocation(mProgram, "vPosition");//핸들이다. 어트리뷰트의 주소를 찾아서 저장
        mColor_u = GLES20.glGetAttribLocation(mProgram, "u_Color");//이것도 핸들이다. attribute 주소를 찾아서 저장
        uMVPMatrixHandle = GLES20.glGetUniformLocation(mProgram, "uMVPMatrix");//이것도 핸들이다. 유니폼의 주소를 찾아서 저장.
    }

    public void draw(float[] vpMatrix){
        //여기서 핸들을 가지고 데이터를 넘겨준대.
        GLES20.glUseProgram(mProgram);//위에서 쓰더니 또쓰네

        GLES20.glEnable(GLES20.GL_BLEND);//attribute는 활성화 시켜야 반영된다는데 뭐가 활성화 된건지
        //블렌딩 기능 활성화
        //블렌딩은 색상 버퍼에 이미 기록되어 있는 값과 새로 그려지는 값의 논리연산 방법을 지정
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);
        //GL_SRC_ALPHA : (AR AR AR AR), GL_ONE_MINUS_SRC_ALPHA : (1-AS, 1-AS, 1-AS, 1-AS) 라는데 이게 뭐야
        //아무튼 색

        GLES20.glEnableVertexAttribArray(mPosition);
        //이 속성을 활성화시켜야 렌더링 시 반영되어 그려진다.
        GLES20.glVertexAttribPointer(mPosition, 3, GLES20.GL_FLOAT, false, COORDS_PER_VERTEX * FLOAT_SIZE, vertexBuffer);
        //이걸로 아까 만든 핸들(mPosition)에 플로트 버퍼 전달 // 어디, 좌표 몇개로 표현, 형식, 그냥 false, 버택스 하나가 총 몇바이트인지(3*4), 그리고 버퍼

        GLES20.glUniformMatrix4fv(uMVPMatrixHandle, 1, false, vpMatrix, 0);
        //이것도 핸들에 vpMatrix전달이니?
        //offset : 몇 번째부터 시작해서 데이터를 넣어줄꺼냐

        GLES20.glEnableVertexAttribArray(mColor_u);
        GLES20.glVertexAttribPointer(mColor_u, 4, GLES20.GL_FLOAT, false, 16, colorBuffer);
        //삼각형은 단색으로 했지만 얘는 attribute로 함. RGB알파 4가지


        if(vertexBuffer.remaining() >= 12){
            GLES20.glDrawElements(GLES20.GL_TRIANGLES, drawOrder.length, GLES20.GL_UNSIGNED_SHORT, drawListBuffer);//요건 뭐니
            //DrawArray 3개씩 해서 하나하나 그려감 - 예제 삼각형 사용
            //DrawElements 버텍스에 정해진 인덱스에 따라서 - 예제 큐브 사용
            //이게 평면 그리는 거인듯. 12 이상이여야 평면이 그려짐. 4개 뭉쳐서
        }

        GLES20.glDisableVertexAttribArray(mPosition);//비활성화
        GLES20.glDisableVertexAttribArray(mColor_u);//비활성화

    }
}
