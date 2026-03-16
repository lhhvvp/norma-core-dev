import React from 'react';
import { Outlet } from 'react-router-dom';
import Navigation from './Navigation';

const MainLayout: React.FC = () => {
  return (
    <div className="w-full h-screen flex flex-col">
      <Navigation />
      <Outlet />
    </div>
  );
};

export default MainLayout;