import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App.tsx'
import './api/websocket.ts'

createRoot(document.getElementById('root')!).render(
  <App />
)
