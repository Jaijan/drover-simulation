# Frontend modules

The browser application uses React, TypeScript, Vite, Three.js, React Three Fiber, Drei, and Plotly. The viewport loads the supplied model from `public/models/drover.glb` (override with `VITE_DROVER_GLB_URL`). Vehicle pose comes from simulation telemetry. The GLB should have its origin at the frame reference point and be oriented consistently with the documented body axes; any required asset-axis alignment should be explicit and configurable.
