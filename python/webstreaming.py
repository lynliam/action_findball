import uvicorn
from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import HTMLResponse, StreamingResponse
import cv2
import numpy as np

app = FastAPI()

# 用于存储客户端WebSocket连接的列表
client_websockets = []

# 客户端WebSocket连接的路由
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    client_websockets.append(websocket)
    try:
        while True:
            # 接收并忽略来自客户端的所有消息
            await websocket.receive()
    finally:
        # 从列表中删除已断开的WebSocket连接
        client_websockets.remove(websocket)

# 用于接收客户端上传的视频流并推送到所有WebSocket连接
@app.post("/upload")
async def upload_video(request: Request):
    # 从请求正文中读取视频流
    content = await request.body()
    nparr = np.frombuffer(content, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    # 将视频帧转换为JPEG格式
    ret, buffer = cv2.imencode('.jpg', frame)
    jpg_as_text = buffer.tobytes()

    # 推送视频帧到所有WebSocket连接
    for websocket in client_websockets:
        await websocket.send_bytes(jpg_as_text)

    # 返回成功响应
    return {"status": "ok"}

# 用于提供前端查看视频的HTML页面
@app.get("/")
async def video_viewer():
    return HTMLResponse("""
        <html>
            <head>
                <title>Video Viewer</title>
            </head>
            <body>
                <h1>Real-time Video Viewer</h1>
                <img id="video_frame" src="#" alt="Video Feed" width="640" height="480"/>
                <script>
                    var video_frame = document.getElementById("video_frame");
                    var ws = new WebSocket("ws://" + window.location.host + "/ws");
                    ws.binaryType = "arraybuffer";
                    ws.onmessage = function(event) {
                        var arrayBuffer = event.data;
                        var blob = new Blob([arrayBuffer], {type: "image/jpeg"});
                        var url = URL.createObjectURL(blob);
                        video_frame.src = url;
                    };
                </script>
            </body>
        </html>
    """)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)

