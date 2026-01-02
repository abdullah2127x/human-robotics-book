import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { ChatWidget } from '../components/ChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        ther eis the chat widget
      </OriginalLayout>
      <ChatWidget />
        ther eis the chat widge2
    </>
  );
}