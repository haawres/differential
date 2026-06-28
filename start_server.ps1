# ==========================================================================
# KEMS UAV Local Static Server
# Runs natively in PowerShell without Node.js or Python dependencies.
# ==========================================================================

$port = 8000
$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add("http://localhost:$port/")

try {
    $listener.Start()
    Write-Host "=======================================================" -ForegroundColor Green
    Write-Host " [SUCCESS] KEMS UAV Local Server Started!" -ForegroundColor Green
    Write-Host " Serving files from: $PWD" -ForegroundColor Cyan
    Write-Host " Local Address:      http://localhost:$port/" -ForegroundColor Yellow
    Write-Host " Press [Ctrl + C] in this window to stop the server." -ForegroundColor Red
    Write-Host "=======================================================" -ForegroundColor Green
    
    # Launch default web browser
    Start-Process "http://localhost:$port/"

    while ($listener.IsListening) {
        $context = $listener.GetContext()
        $request = $context.Request
        $response = $context.Response

        $url = $request.Url.LocalPath
        if ($url -eq "/") { $url = "/index.html" }

        # Resolve physical file path
        $filePath = Join-Path $PWD $url

        # Check if file exists
        if (Test-Path $filePath -PathType Leaf) {
            $bytes = [System.IO.File]::ReadAllBytes($filePath)
            
            # Map correct Content-Type (MIME type)
            $ext = [System.IO.Path]::GetExtension($filePath).ToLower()
            $mime = "text/plain"
            if ($ext -eq ".html") { $mime = "text/html" }
            elseif ($ext -eq ".css") { $mime = "text/css" }
            elseif ($ext -eq ".js") { $mime = "text/javascript" }
            elseif ($ext -eq ".png") { $mime = "image/png" }
            elseif ($ext -eq ".jpg" -or $ext -eq ".jpeg") { $mime = "image/jpeg" }
            elseif ($ext -eq ".mp4") { $mime = "video/mp4" }
            
            $response.ContentType = $mime
            $response.ContentLength64 = $bytes.Length
            $response.OutputStream.Write($bytes, 0, $bytes.Length)
        } else {
            # File not found - return 404
            $response.StatusCode = 404
            $errBytes = [System.Text.Encoding]::UTF8.GetBytes("404 Not Found: $url")
            $response.ContentLength64 = $errBytes.Length
            $response.OutputStream.Write($errBytes, 0, $errBytes.Length)
        }
        $response.OutputStream.Close()
    }
} catch {
    Write-Error $_
} finally {
    $listener.Close()
}
