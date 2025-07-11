# Add this function before if __name__ == "__main__":
def start_web_server():
    """Start web server in background thread"""
    try:
        import uvicorn
        from app import app
        print("🌐 Starting web interface on http://localhost:8000")
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
    except ImportError:
        print("⚠️ Web interface not available (app.py not found)")
    except Exception as e:
        print(f"⚠️ Web interface error: {e}")

# Add these lines at the start of your if __name__ == "__main__": block
if __name__ == "__main__":
    enable_web = "--no-web" not in sys.argv
    
    # ... your existing camera setup code ...
    
    # Add this before starting hand thread:
    if enable_web:
        web_thread = threading.Thread(target=start_web_server, daemon=True)
        web_thread.start()
        print("🌐 Web interface will be available at: http://localhost:8000")
    
    # ... rest of your existing code stays the same ...
