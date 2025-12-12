import React from 'react';
import Particles from 'react-tsparticles';
import { loadSlim } from 'tsparticles-slim';
import { useCallback, useMemo } from 'react';

const BackgroundAnimation = () => {
  const options = useMemo(() => {
    return {
      background: {
        color: '#000', // Dark background for subtle effect
      },
      interactivity: {
        events: {
          onClick: {
            enable: true,
            mode: 'push',
          },
          onHover: {
            enable: true,
            mode: 'repulse',
          },
        },
        modes: {
          push: {
            quantity: 10,
          },
          repulse: {
            distance: 100,
          },
        },
      },
      particles: {
        links: {
          enable: false, // No lines between particles
        },
        move: {
          enable: true,
          speed: { min: 0.1, max: 0.5 },
        },
        opacity: {
          value: { min: 0.1, max: 0.5 },
        },
        size: {
          value: { min: 1, max: 3 },
        },
        // Glowing effect
        shadow: {
            enable: true,
            color: '#ffffff', // White glow
            blur: 5,
        },
        twinkle: { // Subtle twinkling effect
            lines: {
                enable: false,
            },
            particles: {
                enable: true,
                frequency: 0.05,
                opacity: 1,
                color: '#ffffff',
            }
        }
      },
    };
  }, []);

  const particlesInit = useCallback((engine: any) => {
    loadSlim(engine);
  }, []);

  return (
    <Particles
      id="tsparticles"
      init={particlesInit}
      options={options as any}
      style={{
        position: 'absolute',
        width: '100%',
        height: '100%',
        top: 0,
        left: 0,
        zIndex: -1, // Ensure it's behind other content
      }}
    />
  );
};

export default BackgroundAnimation;