import React from 'react';
import Layout from '@theme/Layout';

export default function Home() {
  return (
    <Layout title="Physical AI Book">
      <div
        style={{
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          height: '100vh',
          background: '#e0f7fa',
          color: '#00796b',
          textAlign: 'center',
          padding: '20px',
          fontFamily: "'Poppins', sans-serif",
        }}
      >
        <h1 style={{
          fontSize: '3rem',
          marginBottom: '20px',
          fontWeight: '700',
        }}>
          Welcome to Physical AI Book 📘
        </h1>
        <p style={{
          fontSize: '1.5rem',
          marginBottom: '40px',
          color: '#004d40',
        }}>
          Your documentation website is running successfully!
        </p>
        <a
          href="/docs"
          style={{
            textDecoration: 'none',
            padding: '15px 40px',
            borderRadius: '50px',
            background: 'linear-gradient(90deg, #00bfa5, #1de9b6)',
            color: '#fff',
            fontWeight: 'bold',
            fontSize: '1.2rem',
            boxShadow: '0 5px 15px rgba(0,0,0,0.2)',
            transition: 'all 0.3s ease',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.transform = 'scale(1.05)';
            e.currentTarget.style.boxShadow = '0 10px 25px rgba(0,0,0,0.3)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.transform = 'scale(1)';
            e.currentTarget.style.boxShadow = '0 5px 15px rgba(0,0,0,0.2)';
          }}
        >
          Go to docs →
        </a>
      </div>
    </Layout>
  );
}
