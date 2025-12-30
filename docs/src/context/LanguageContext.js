import React, { createContext, useContext, useState, useEffect } from 'react';

// Define translations for different languages
const translations = {
  en: {
    language: 'Language',
    english: 'English',
    urdu: 'Urdu',
    contentInEnglish: 'Content will be displayed in English',
    contentInUrdu: 'متن اردو میں دکھایا جائے گا',
    chatPlaceholder: 'Ask a question about this chapter...',
    send: 'Send',
    textbookAssistant: 'Textbook Assistant',
    personalizationLevel: 'Personalization Level',
    defaultContent: 'Default Content',
    beginnerFriendly: 'Beginner-Friendly',
    intermediate: 'Intermediate',
    advanced: 'Advanced',
    contentSimplified: 'Content will be simplified with additional explanations',
    contentStandard: 'Content will be presented at standard level',
    contentAdvanced: 'Content will include additional technical details',
    originalContent: 'Content will be presented as originally written',
    createAccount: 'Create Your Account',
    email: 'Email',
    fullName: 'Full Name',
    softwareBackground: 'Software Background',
    hardwareBackground: 'Hardware Background',
    signUp: 'Sign Up',
    creatingAccount: 'Creating Account...',
    profileManagement: 'Profile Management',
    editProfile: 'Edit Profile',
    saveChanges: 'Save Changes',
    saving: 'Saving...',
    cancel: 'Cancel',
    profileUpdated: 'Profile updated successfully!',
    failedUpdate: 'Failed to update profile. Please try again.',
    welcomeTitle: 'Welcome to Physical AI & Humanoid Robotics',
    beforeBegin: 'Before You Begin',
    personalizeExperience: 'To get the most personalized learning experience, please tell us about your background. This will help us tailor the content to your experience level.',
    whatYoullLearn: 'What You\'ll Learn',
    howToUse: 'How to Use This Textbook'
  },
  ur: {
    language: 'زبان',
    english: 'انگریزی',
    urdu: 'اردو',
    contentInEnglish: 'مواد انگریزی میں ظاہر ہوگا',
    contentInUrdu: 'متن اردو میں دکھایا جائے گا',
    chatPlaceholder: 'اس باب کے بارے میں ایک سوال پوچھیں...',
    send: 'بھیجیں',
    textbookAssistant: 'میٹھاکتاب کے اسسٹنٹ',
    personalizationLevel: 'شخصیت کی سطح',
    defaultContent: 'ڈیفالٹ مواد',
    beginnerFriendly: 'بلا مہارت والا دوست',
    intermediate: 'درمیانہ',
    advanced: 'اعلی درجے کا',
    contentSimplified: 'مواد کو مزید وضاحت کے ساتھ آسان بنادیا جائے گا',
    contentStandard: 'مواد کو معیاری سطح پر پیش کیا جائے گا',
    contentAdvanced: 'مواد میں اضافی تکنیکی تفصیلات شامل کی جائیں گی',
    originalContent: 'مواد کو اصلی طور پر پیش کیا جائے گا',
    createAccount: 'اکاؤنٹ بنائیں',
    email: 'ای میل',
    fullName: 'مکمل نام',
    softwareBackground: 'سافٹ ویئر کا پس منظر',
    hardwareBackground: 'ہارڈ ویئر کا پس منظر',
    signUp: 'سائن اپ کریں',
    creatingAccount: 'اکاؤنٹ بن رہا ہے...',
    profileManagement: 'پروفائل کا نظم',
    editProfile: 'پروفائل میں ترمیم کریں',
    saveChanges: 'تبدیلیاں محفوظ کریں',
    saving: 'محفوظ کر رہا ہے...',
    cancel: 'منسوخ کریں',
    profileUpdated: 'پروفائل کامیابی سے اپ ڈیٹ ہو گیا!',
    failedUpdate: 'پروفائل کو اپ ڈیٹ کرنے میں ناکامی ہوئی. براے مہربانی دوبارہ کوشش کریں.',
    welcomeTitle: 'فزیکل ای آئی اور ہیومنوڈ روبوٹکس میں خوش آمدید',
    beforeBegin: 'شروع کرنے سے پہلے',
    personalizeExperience: 'سیکھنے کا تجربہ زیادہ سے زیادہ ذاتی نوعیت کا حاصل کرنے کے لیے، براہ کرم ہمیں اپنے پس منظر کے بارے میں بتائیں. یہ ہمیں مواد کو آپ کی تجربے کی سطح کے مطابق ترتیب دینے میں مدد کرے گا.',
    whatYoullLearn: 'آپ کیا سیکھیں گے',
    howToUse: 'اس میٹھاکتاب کو کیسے استعمال کریں'
  }
};

// Create context
const LanguageContext = createContext();

// Provider component
export const LanguageProvider = ({ children }) => {
  const [language, setLanguage] = useState(() => {
    // Check if language preference is stored in localStorage
    const savedLanguage = localStorage.getItem('preferredLanguage');
    return savedLanguage || 'en';
  });

  // Update localStorage when language changes
  useEffect(() => {
    localStorage.setItem('preferredLanguage', language);
  }, [language]);

  // Function to switch language
  const switchLanguage = (newLanguage) => {
    setLanguage(newLanguage);
  };

  // Get translation for current language
  const t = (key) => {
    const langTranslations = translations[language] || translations.en;
    return langTranslations[key] || key;
  };

  return (
    <LanguageContext.Provider value={{ language, switchLanguage, t }}>
      {children}
    </LanguageContext.Provider>
  );
};

// Custom hook to use the language context
export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};