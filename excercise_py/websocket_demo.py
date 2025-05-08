import websockets
import asyncio


async def handler(websocket, path):
    async for message in websocket:
        await websocket.send("Server received: " + message)


start_server = websockets.serve(handler, "localhost", 1234)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
