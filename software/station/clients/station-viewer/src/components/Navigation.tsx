import { Link, useLocation } from "react-router-dom";

function Navigation() {
  const location = useLocation();

  const isActive = (path: string) => {
    return location.pathname === path;
  };

  return (
    <div className="relative z-10 bg-gray-900 border-b-2 border-gray-700">
      <div className="px-4 py-2 flex items-center gap-4">
        <Link to="/" className="group">
          <img
            src="/logo.svg"
            alt="Station View"
            title="NormaCore"
            className="h-8 logo-first-load opacity-80 transition-opacity duration-200 group-hover:opacity-100"
          />
        </Link>
        <nav className="flex gap-4">
          <Link
            to="/"
            className={`px-3 py-1 rounded text-sm font-medium transition-colors ${
              isActive("/")
                ? "bg-green-600 text-white"
                : "text-gray-300 hover:text-white hover:bg-gray-700"
            }`}
          >
            Home
          </Link>
          <Link
            to="/history"
            className={`px-3 py-1 rounded text-sm font-medium transition-colors ${
              isActive("/history")
                ? "bg-green-600 text-white"
                : "text-gray-300 hover:text-white hover:bg-gray-700"
            }`}
          >
            History
          </Link>
        </nav>

        <span
          className="ml-auto text-[11px] font-mono text-gray-500"
          title="Station build version and commit hash"
        >
          {`⎇ ${__STATION_VERSION__}`}
        </span>
      </div>
    </div>
  );
}

export default Navigation;
