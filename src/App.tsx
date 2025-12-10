// frontend/src/App.tsx


import { BookReader } from './components/Book/BookReader';
import { ChatWidget } from './components/Chat/ChatWidget';
import './App.css';

function App() {
  return (
    <div className="App">
      <BookReader />
      <ChatWidget />
    </div>
  );
}

export default App;