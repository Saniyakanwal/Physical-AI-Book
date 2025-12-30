import React from 'react';
import { LanguageProvider } from './context/LanguageContext';

export default function Root({ children }) {
  return <LanguageProvider>{children}</LanguageProvider>;
}