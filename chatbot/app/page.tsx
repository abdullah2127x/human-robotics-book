'use client';

import { useState } from 'react';
import ChatWidget from './components/ChatWidget';

export default function Home() {
  const [activeSection] = useState('chat');

  return (
    <div className="min-h-screen bg-gray-900 text-gray-100">
      <div className="container mx-auto px-4 py-8">
        <h1 className="text-2xl font-bold mb-4 text-center">Chat Interface Test</h1>
        <p className="text-center text-gray-400">This is a minimal page for chat functionality testing.</p>

        <div className="mt-8 text-center">
          <p>Select any text on this page to test the context feature.</p>
          <p className="mt-2"><strong>This text can be selected</strong> to test the chat widget.</p>
        </div>
      </div>

      <ChatWidget activeSection={activeSection} />
    </div>
  );
}