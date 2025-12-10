export interface Chapter {
  id: number;
  number: number;
  title: string;
  part: 1 | 2 | 3;
  content: string;
  pageStart: number;
  pageEnd: number;
}

export interface Page {
  id: number;
  pageNumber: number;
  chapterNumber: number;
  content: string;
  part: 1 | 2 | 3;
}

export interface BookState {
  currentPage: number;
  totalPages: number;
  chapters: Chapter[];
}