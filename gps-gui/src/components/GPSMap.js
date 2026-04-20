import useROSSubscription from '../useROSSubscription';
import React, { useEffect, useRef, useState } from 'react';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';

const GPSMap = () => {
  const mapContainerRef = useRef(null);
  const mapRef = useRef(null);
  const markerRef = useRef(null);
  const polylineRef = useRef(null);
  const pathCoordsRef = useRef([]);

  const [lastFix, setLastFix] = useState(null);
  const [pathLength, setPathLength] = useState(0);
  const [followPlane, setFollowPlane] = useState(true);

  // Connection to Jetson
const gpsMsg = useROSSubscription('/fix', 'sensor_msgs/NavSatFix');

  // ── Initialize map ──
  useEffect(() => {
    if (mapRef.current) return;
    mapRef.current = L.map(mapContainerRef.current).setView([40.7128, -74.0060], 15);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors',
      maxZoom: 19
    }).addTo(mapRef.current);
  }, []);

  // ── Update map when GPS message arrives ──
  useEffect(() => {
    if (!gpsMsg || !mapRef.current) return;

    const { latitude: lat, longitude: lon, altitude, status } = gpsMsg;
    const hasFix = status && status.status >= 0;

    setLastFix({ lat, lon, altitude, hasFix });

    if (!hasFix) return;

    const latlng = [lat, lon];

    // Add to path
    pathCoordsRef.current.push(latlng);
    if (pathCoordsRef.current.length > 1000) pathCoordsRef.current.shift();
    setPathLength(pathCoordsRef.current.length);

    // Calculate heading
    let heading = 0;
    const coords = pathCoordsRef.current;
    if (coords.length >= 2) {
      const prev = coords[coords.length - 2];
      const toRad = d => d * Math.PI / 180;
      const dLon = toRad(lon - prev[1]);
      const y = Math.sin(dLon) * Math.cos(toRad(lat));
      const x = Math.cos(toRad(prev[0])) * Math.sin(toRad(lat)) -
                Math.sin(toRad(prev[0])) * Math.cos(toRad(lat)) * Math.cos(dLon);
      heading = ((Math.atan2(y, x) * 180 / Math.PI) + 360) % 360;
    }

    // Plane icon
    const planeIcon = L.divIcon({
      className: '',
      iconSize: [40, 40],
      iconAnchor: [20, 20],
      html: `
        <div style="transform:rotate(${heading}deg);width:40px;height:40px;
                    display:flex;align-items:center;justify-content:center;">
          <svg width="40" height="40" viewBox="0 0 40 40" fill="none">
            <path d="M20 4 L24 18 L36 22 L24 24 L22 36 L20 32 L18 36 L16 24 L4 22 L16 18 Z"
                  fill="#e87820" fill-opacity="0.95"/>
            <path d="M20 4 L24 18 L36 22 L24 24 L22 36 L20 32 L18 36 L16 24 L4 22 L16 18 Z"
                  fill="none" stroke="#ffffff" stroke-width="1" opacity="0.6"/>
          </svg>
        </div>`
    });

    // Create or update marker
    if (!markerRef.current) {
      markerRef.current = L.marker(latlng, { icon: planeIcon, zIndexOffset: 1000 })
        .addTo(mapRef.current);
    } else {
      markerRef.current.setLatLng(latlng);
      markerRef.current.setIcon(planeIcon);
    }

    // Create or update path
    if (!polylineRef.current) {
      polylineRef.current = L.polyline(pathCoordsRef.current, {
        color: '#e87820', weight: 2, opacity: 0.7
      }).addTo(mapRef.current);
    } else {
      polylineRef.current.setLatLngs(pathCoordsRef.current);
    }

    // Follow plane
    if (followPlane) {
      mapRef.current.panTo(latlng, { animate: true, duration: 0.5 });
    }

  }, [gpsMsg, followPlane]);

  // ── Clear path ──
  const handleClearPath = () => {
    pathCoordsRef.current = [];
    setPathLength(0);
    if (polylineRef.current && mapRef.current) {
      mapRef.current.removeLayer(polylineRef.current);
      polylineRef.current = null;
    }
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100vh', background: '#1a1a1a', fontFamily: 'sans-serif' }}>

      {/* Title bar */}
      <div style={{ background: '#b16200', color: '#fff', padding: '10px 20px',
                    display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <span style={{ fontSize: '18px', fontWeight: 'bold', letterSpacing: '2px' }}>
          ICARUS — GPS FLIGHT TRACKER
        </span>
        <span style={{ fontSize: '12px', opacity: 0.7 }}>
          {lastFix ? (lastFix.hasFix ? '🟢 FIXED' : '🔴 NO FIX') : '⚪ WAITING'}
        </span>
      </div>

      <div style={{ display: 'flex', flex: 1, overflow: 'hidden' }}>

        {/* Sidebar */}
        <div style={{ width: '220px', background: '#111', padding: '16px',
                      borderRight: '1px solid #333', display: 'flex',
                      flexDirection: 'column', gap: '12px' }}>

          <div>
            <div style={{ fontSize: '10px', color: '#666', letterSpacing: '2px',
                          textTransform: 'uppercase', marginBottom: '8px' }}>Position</div>
            <div style={{ fontFamily: 'monospace', fontSize: '13px', color: '#e87820', lineHeight: '1.8' }}>
              <div>LAT: {lastFix ? lastFix.lat.toFixed(6) + '°' : '—'}</div>
              <div>LON: {lastFix ? lastFix.lon.toFixed(6) + '°' : '—'}</div>
              <div>ALT: {lastFix ? lastFix.altitude.toFixed(1) + ' m' : '—'}</div>
            </div>
          </div>

          <div>
            <div style={{ fontSize: '10px', color: '#666', letterSpacing: '2px',
                          textTransform: 'uppercase', marginBottom: '8px' }}>Path</div>
            <div style={{ fontFamily: 'monospace', fontSize: '13px', color: '#e87820' }}>
              {pathLength} points
            </div>
          </div>

          <button onClick={() => setFollowPlane(f => !f)}
            style={{ padding: '8px', background: 'transparent', cursor: 'pointer',
                     border: `1px solid ${followPlane ? '#e87820' : '#333'}`,
                     color: followPlane ? '#e87820' : '#666',
                     fontSize: '12px', letterSpacing: '1px', textTransform: 'uppercase',
                     borderRadius: '4px' }}>
            {followPlane ? '⊙ Following' : '○ Follow Off'}
          </button>

          <button onClick={handleClearPath}
            style={{ padding: '8px', background: 'transparent', cursor: 'pointer',
                     border: '1px solid #330011', color: '#ff3355',
                     fontSize: '12px', letterSpacing: '1px', textTransform: 'uppercase',
                     borderRadius: '4px' }}>
            ✕ Clear Path
          </button>

        </div>

        {/* Map */}
        <div style={{ flex: 1 }}>
          <div ref={mapContainerRef} style={{ width: '100%', height: '100%' }} />
        </div>

      </div>
    </div>
  );
};

export default GPSMap;