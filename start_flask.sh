echo "Starting flask server..."
sleep 15
cd ~/Documents/projects/heizungsregelung
gunicorn --bind 0.0.0.0:8008 flask_server:app
