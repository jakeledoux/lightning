import uvicorn
from vidgear.gears.asyncio import WebGear

web = WebGear(source='grant.mp4')

uvicorn.run(web(), host='0.0.0.0', port=8080)

web.shutdown()
