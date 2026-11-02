"""LAN entry point for the real API. Forwards once; never retries motion requests."""
import argparse
from contextlib import asynccontextmanager


def create_app(upstream='http://127.0.0.1:8795', transport=None):
    import httpx
    from fastapi import FastAPI, Request
    from starlette.responses import JSONResponse, Response

    client = httpx.AsyncClient(base_url=upstream, trust_env=False, timeout=60., transport=transport)

    @asynccontextmanager
    async def lifespan(app):
        try:
            yield
        finally:
            await client.aclose()

    app = FastAPI(openapi_url=None, docs_url=None, redoc_url=None, lifespan=lifespan)
    excluded = {'host', 'connection', 'transfer-encoding', 'content-length', 'content-encoding',
                'keep-alive', 'te', 'trailer', 'upgrade', 'proxy-authorization', 'proxy-authenticate'}

    async def forward(request):
        body = bytearray()
        async for chunk in request.stream():
            body.extend(chunk)
            if len(body) > 1024 * 1024:
                return JSONResponse({'error': 'Request too large'}, status_code=413)
        headers = {k: v for k, v in request.headers.items() if k.lower() not in excluded}
        path = request.url.path
        if request.url.query:
            path += '?' + request.url.query
        try:
            response = await client.request(request.method, path, content=bytes(body), headers=headers)
        except httpx.RequestError:
            return JSONResponse({'code': 2004, 'message': 'Real executor unavailable or outcome unknown; '
                                 'query the same taskId before resubmitting. No retry was sent.'}, status_code=502)
        return Response(response.content, status_code=response.status_code,
                        headers={k: v for k, v in response.headers.items() if k.lower() not in excluded})

    # Annotation must resolve locally on Python 3.8 without global FastAPI imports.
    forward.__annotations__['request'] = Request
    app.add_api_route('/{path:path}', forward, methods=['GET', 'POST', 'PUT', 'DELETE', 'PATCH', 'OPTIONS', 'HEAD'])
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--upstream', default='http://127.0.0.1:8795')
    args = parser.parse_args()
    import uvicorn
    uvicorn.run(create_app(args.upstream), host=args.host, port=args.port, workers=1)
